//! ELRS radio inputs switcher. It acts like a proxy switching between multiple radio inputs.
//!
//! This assumes that input 1 is connected to A10 pin, input 2 to A3 pin and output to B10 pin.
//! By default it listens to AUX4 value and when it changes corresponding input becomes active.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};
use crsf::{OwnedRawPacket, Packet, PacketReader as CrsfPacketReader, PacketPayload};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::{Channel as ExtiChannel, ExtiInput};
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::{USART1, USART2, USART3, DMA1_CH2, DMA1_CH4};
use embassy_stm32::usart::{Config as UartConfig, InterruptHandler, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_probe as _;

#[path = "../common.rs"]
mod common;
use common::*;

const CRSF_BAUDRATE: u32 = 420_000;

static SWITCH_MODE: AtomicU8 = AtomicU8::new(SwitchMode::Independent as u8);

static ACTIVE_RX: AtomicU8 = AtomicU8::new(0);

static PACKETS_QUEUE: Channel<ThreadModeRawMutex, OwnedRawPacket, 4> = Channel::new();

#[repr(u8)]
#[derive(Clone, Copy)]
enum SwitchMode {
    Independent,
    // We construct it from u8 value
    #[allow(dead_code)]
    Simultaneous,
}

impl SwitchMode {
    fn next_mode(&self) -> Self {
        Self::from(*self as u8 + 1)
    }
}

impl From<u8> for SwitchMode {
    fn from(source: u8) -> Self {
        SwitchMode::try_from(source).unwrap_or(SwitchMode::Independent)
    }
}

#[embassy_executor::task]
async fn blink(led: AnyPin) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    led.set_low();

    loop {
        // Indicate current switch mode with long blinks
        common::blink(&mut led, SWITCH_MODE.load(Ordering::Relaxed), 1000).await;

        // Indicate current aux channel with short blinks
        common::blink(&mut led, AUX_SWITCH, 200).await;
        Timer::after_millis(2000).await;
    }
}

#[embassy_executor::task]
async fn mode_switcher(mut switch_mode_input: ExtiInput<'static, AnyPin>) {
    loop {
        switch_mode_input.wait_for_rising_edge().await;
        SWITCH_MODE.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| {
            Some(SwitchMode::from(v).next_mode() as u8)
        }).ok();
    }
}

#[embassy_executor::task]
async fn read_rx1(
    uart: UartRx<'static, USART1, DMA1_CH2>,
    input_num: u8,
) {
    read_input(AnyUartRx::Uart1 { uart }, input_num).await
}

#[embassy_executor::task]
async fn read_rx2(
    uart: UartRx<'static, USART2, DMA1_CH4>,
    input_num: u8,
) {
    read_input(AnyUartRx::Uart2 { uart }, input_num).await
}

async fn read_input(
    mut uart: AnyUartRx,
    input_num: u8,
) {
    let mut prev_switch_state = -1i8;

    let mut parser = CrsfPacketReader::default();
    let mut buf = [0u8; Packet::MAX_LENGTH];
    let mut num_uart_errors = 0;
    let mut num_parser_errors = 0;
    loop {
        match uart.read_until_idle(&mut buf).await {
            Ok(0) => {}
            Ok(n) => {
                let mut buf = &buf[..n];
                while !buf.is_empty() {
                    let (raw_packet, consumed) = parser.push_bytes(&buf);
                    buf = &buf[consumed..];
                    if let Some(raw_packet) = raw_packet {
                        match Packet::parse(raw_packet) {
                            Ok(Packet { addr: _, payload: PacketPayload::RcChannels(channels) }) => {
                                let switch_val = channels[SWITCH_CHANNEL_IX];
                                let cur_switch_state = if switch_val > TRIGGER_VAL { 1 } else { 0 };

                                match SWITCH_MODE.load(Ordering::Relaxed).into() {
                                    SwitchMode::Independent => {
                                        if prev_switch_state < 0 {
                                            prev_switch_state = cur_switch_state;
                                        } else if prev_switch_state != cur_switch_state {
                                            ACTIVE_RX.store(input_num, Ordering::Relaxed);
                                            prev_switch_state = cur_switch_state;
                                        }
                                    }
                                    SwitchMode::Simultaneous => {
                                        if cur_switch_state != prev_switch_state {
                                            ACTIVE_RX.store(cur_switch_state as u8, Ordering::Relaxed);
                                            prev_switch_state = cur_switch_state;
                                        }
                                    }
                                }
                            }
                            Ok(_) => {}
                            Err(e) => {
                                if num_parser_errors % 100 == 0 {
                                    error!("Error when parsing CRSF packet: {}", e);
                                }
                                num_parser_errors += 1;
                            }
                        }

                        if ACTIVE_RX.load(Ordering::Relaxed) == input_num {
                            PACKETS_QUEUE.send(raw_packet.to_owned()).await;
                        }
                    }
                }
            }
            Err(e) => {
                if num_uart_errors % 1000 == 0 {
                    error!("Error reading RX uart: {}", e);
                }
                parser.reset();
                num_uart_errors += 1;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(mcu_config());
    info!("Starting ELRS real diversity (aircraft side)");

    bind_interrupts!(struct Irqs {
        USART1 => InterruptHandler<USART1>;
        USART2 => InterruptHandler<USART2>;
        USART3 => InterruptHandler<USART3>;
    });
    let uart_config = {
        let mut cfg = UartConfig::default();
        cfg.baudrate = CRSF_BAUDRATE;
        cfg
    };
    let rx1_uart = UartRx::new(
        p.USART1, Irqs, p.PA10, p.DMA1_CH2, uart_config
    ).unwrap();
    let rx2_uart = UartRx::new(
        p.USART2, Irqs, p.PA3, p.DMA1_CH4, uart_config
    ).unwrap();
    let mut fc_uart = UartTx::new(
        p.USART3, p.PB10, p.DMA1_CH5, uart_config
    ).unwrap();

    let switch_mode_input = ExtiInput::new(
        Input::new(p.PC13, Pull::Down).degrade(),
        p.EXTI13.degrade()
    );

    spawner.spawn(
        mode_switcher(switch_mode_input)
    ).unwrap();
    spawner.spawn(
        blink(p.PC6.degrade())
    ).unwrap();
    spawner.spawn(
        read_rx1(rx1_uart, 0)
    ).unwrap();
    spawner.spawn(
        read_rx2(rx2_uart, 1)
    ).unwrap();

    loop {
        let raw_packet = PACKETS_QUEUE.receive().await;
        if let Err(e) = fc_uart.write(&raw_packet.as_slice()).await {
            error!("Error writing to uart: {}", e);
        }
    }
}
