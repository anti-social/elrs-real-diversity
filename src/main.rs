//! Swither of radio inputs. It acts like a proxy switching between multiple radio inputs.
//!
//! This assumes that input 1 is connected to A10 pin, input 2 to A3 pin and output to B10 pin.
//! By default it listens to AUX4 value and when it changes corresponding input becomes active.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};
use crsf::{CrsfPacketParser, DefaultChannelsMapper, Packet, RcChannelsMapped, RawPacket};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Speed, Pull};
use embassy_stm32::peripherals::{DMA1_CH5, DMA1_CH6, PA5, USART1, USART2, USART3};
use embassy_stm32::usart::{Config as UartConfig, Error as UartError, InterruptHandler, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_probe as _;

const CRSF_BAUDRATE: u32 = 420_000;

static ACTIVE_INPUT: AtomicU8 = AtomicU8::new(0);
// AUX 4 = SD switch on Radiomaster by default
static AUX_SWITCH: AtomicU8 = AtomicU8::new(4);
static CONTROL_CHANNELS: u8 = 4;
static TRIGGER_VAL: i32 = 1400;

static PACKETS_QUEUE: Channel<ThreadModeRawMutex, RawPacket, 4> = Channel::new();

#[embassy_executor::task]
async fn blink(led: AnyPin) {
    let mut led = Output::new(led, Level::High, Speed::Low);

    led.set_high();
    loop {
        // Number of short blinks is equal to number of AUX
        let aux_switch = AUX_SWITCH.load(Ordering::Relaxed);
        for _ in 0..aux_switch {
            led.set_low();
            Timer::after_millis(200).await;
            led.set_high();
            Timer::after_millis(200).await;
        }

        Timer::after_millis(2000).await;
    }
}

#[embassy_executor::task]
async fn switch_aux(mut button: ExtiInput<'static, PA5>) {
    loop {
        button.wait_for_rising_edge().await;
        Timer::after_millis(10).await;
        if button.is_low() {
            continue;
        }
        button.wait_for_falling_edge().await;
        AUX_SWITCH.fetch_update(
            Ordering::Relaxed,
            Ordering::Relaxed,
            |aux_switch| {
                Some(if aux_switch >= 12 {
                    1
                } else {
                    aux_switch + 1
                })
            }
        ).ok();
    }
}

enum AnyUartRx {
    Uart1 { port: UartRx<'static, USART1, DMA1_CH5> },
    Uart2 { port: UartRx<'static, USART2, DMA1_CH6> },
}

impl AnyUartRx {
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        use AnyUartRx::*;

        match self {
            Uart1 { port } => port.read_until_idle(buf).await,
            Uart2 { port } => port.read_until_idle(buf).await,
        }
    }
}

#[embassy_executor::task]
async fn read_input_1(
    uart: AnyUartRx,
    input_num: u8,
) {
    read_input(uart, input_num).await
}

#[embassy_executor::task]
async fn read_input_2(
    uart: AnyUartRx,
    input_num: u8,
) {
    read_input(uart, input_num).await
}

async fn read_input(
    mut uart: AnyUartRx,
    input_num: u8,
) {
    let mut prev_switch_state = -1i8;

    let mut parser = CrsfPacketParser::default();
    let mut buf = [0u8; 64];
    loop {
        match uart.read_until_idle(&mut buf).await {
            Ok(n) if n == 0 => {}
            Ok(n) => {
                parser.push_bytes(&buf[..n]);
                while let Some(Ok(raw_packet)) = parser.next_raw_packet() {
                    let packet = Packet::parse(raw_packet.data());
                    if let Some(Packet::RcChannelsPacked(raw_channels)) = packet {
                        let channels = RcChannelsMapped::<DefaultChannelsMapper>::new(raw_channels);
                        let switch_channel = CONTROL_CHANNELS + AUX_SWITCH.load(Ordering::Relaxed);
                        let switch_val = channels[switch_channel as usize - 1];
                        let cur_switch_state = if switch_val > TRIGGER_VAL { 1 } else { 0 };
                        if prev_switch_state < 0 {
                            prev_switch_state = cur_switch_state;
                        } else if prev_switch_state != cur_switch_state {
                            ACTIVE_INPUT.store(input_num, Ordering::Relaxed);
                            prev_switch_state = cur_switch_state;
                        }
                    }

                    if ACTIVE_INPUT.load(Ordering::Relaxed) == input_num {
                        PACKETS_QUEUE.send(raw_packet).await;
                    }
                }
            }
            Err(e) => {
                warn!("Error reading from uart {}: {}", input_num, e);
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Starting radio switcher");

    bind_interrupts!(struct Irqs {
        USART1 => InterruptHandler<USART1>;
        USART2 => InterruptHandler<USART2>;
        USART3 => InterruptHandler<USART3>;
    });
    let mut crsf_uart_config = UartConfig::default();
    crsf_uart_config.baudrate = CRSF_BAUDRATE;
    let in1_uart : UartRx<USART1, DMA1_CH5> = UartRx::new(
        p.USART1, Irqs, p.PA10, p.DMA1_CH5, crsf_uart_config
    ).unwrap();
    let in2_uart = UartRx::new(
        p.USART2, Irqs, p.PA3, p.DMA1_CH6, crsf_uart_config
    ).unwrap();
    let mut out_uart = UartTx::new(
        p.USART3, p.PB10, p.DMA1_CH2, crsf_uart_config
    ).unwrap();
    let switch_aux_button = ExtiInput::new(
        Input::new(p.PA5, Pull::Down), p.EXTI5
    );

    spawner.spawn(blink(p.PC13.degrade())).unwrap();
    spawner.spawn(switch_aux(switch_aux_button)).unwrap();
    spawner.spawn(read_input_1(AnyUartRx::Uart1 { port: in1_uart }, 0)).unwrap();
    spawner.spawn(read_input_2(AnyUartRx::Uart2 { port: in2_uart }, 1)).unwrap();

    loop {
        let raw_packet = PACKETS_QUEUE.receive().await;
        if let Err(e) = out_uart.write(&raw_packet.data()).await {
            error!("Error writing to uart: {}", e);
        }
    }
}
