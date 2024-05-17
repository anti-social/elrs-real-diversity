//! ELRS transmitters switcher. It acts like a proxy switching between multiple transmitters
//! that are connected to a handset.
//!
//! This assumes that input 1 is connected to A10 pin, input 2 to A3 pin and output to B10 pin.
//! By default it listens to AUX4 value and when it changes corresponding input becomes active.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};
use crsf::{OwnedRawPacket, Packet, PacketAddress, PacketReader as CrsfPacketReader, PacketPayload};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Speed};
use embassy_stm32::peripherals::{DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, USART1, USART2, USART3};
use embassy_stm32::usart::{Config as UartConfig, InterruptHandler, Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use panic_probe as _;

#[path = "../common.rs"]
mod common;
use common::*;

const CRSF_BAUDRATE: u32 = 400_000;

static ACTIVE_TX: AtomicU8 = AtomicU8::new(0);

static HANDSET_TO_TX_PACKETS_QUEUE: Channel<ThreadModeRawMutex, OwnedRawPacket, 1> = Channel::new();
static TX_TO_HANDSET_PACKETS_QUEUE: Channel<ThreadModeRawMutex, OwnedRawPacket, 1> = Channel::new();

#[embassy_executor::task]
async fn blink(led: AnyPin) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    led.set_low();

    loop {
        // Indicate current aux channel with short blinks
        common::blink(&mut led, AUX_SWITCH, 200).await;
        Timer::after_millis(2000).await;
    }
}

#[embassy_executor::task]
async fn read_handset_data(
    mut uart: UartRx<'static, USART1, DMA1_CH2>,
) {
    let mut active_tx = ACTIVE_TX.load(Ordering::Acquire);
    let mut parser = CrsfPacketReader::with_addresses(
        &[PacketAddress::Transmitter, PacketAddress::Handset]
    );
    let mut buf = [0; Packet::MAX_LENGTH];
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
                        // Ignore data that we are sending to a handset
                        if raw_packet.addr() != PacketAddress::Transmitter {
                            continue;
                        }

                        match Packet::parse(raw_packet) {
                            Ok(Packet { addr: _, payload: PacketPayload::RcChannels(channels) }) => {
                                let switch_val = channels[SWITCH_CHANNEL_IX];
                                let new_active_tx = if switch_val > TRIGGER_VAL { 1 } else { 0 };
                                if new_active_tx != active_tx {
                                    active_tx = new_active_tx;
                                    ACTIVE_TX.store(active_tx, Ordering::Release);
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

                        HANDSET_TO_TX_PACKETS_QUEUE.try_send(raw_packet.to_owned()).ok();
                    }
                }
            }
            Err(e) => {
                if num_uart_errors % 1000 == 0 {
                    error!("Error reading handset uart: {}", e);
                }
                parser.reset();
                num_uart_errors += 1;
            }
        }
    }
}

#[embassy_executor::task]
async fn read_tx1_data(
    uart: UartRx<'static , USART2, DMA1_CH4>,
) {
    read_tx_data(AnyUartRx::Uart2 { uart }, 0).await
}

#[embassy_executor::task]
async fn read_tx2_data(
    uart: UartRx<'static , USART3, DMA1_CH6>,
) {
    read_tx_data(AnyUartRx::Uart3 { uart }, 1).await
}

async fn read_tx_data(
    mut uart: AnyUartRx,
    tx_ord: u8,
) {
    let mut parser = CrsfPacketReader::with_addresses(
        &[PacketAddress::Transmitter, PacketAddress::Handset]
    );
    let mut buf = [0u8; Packet::MAX_LENGTH];
    let mut num_uart_errors = 0;
    loop {
        match uart.read_until_idle(&mut buf).await {
            Ok(0) => {}
            Ok(n) => {
                let mut buf = &buf[..n];
                while !buf.is_empty() {
                    let (raw_packet, consumed) = parser.push_bytes(&buf);
                    buf = &buf[consumed..];

                    if let Some(raw_packet) = raw_packet {
                        // Another transmitter is active at the moment
                        if ACTIVE_TX.load(Ordering::Acquire) != tx_ord {
                            continue;
                        }

                        // Ignore data that we are sending to a transmitter
                        if raw_packet.addr() != PacketAddress::Handset {
                            continue;
                        }

                        TX_TO_HANDSET_PACKETS_QUEUE.try_send(raw_packet.to_owned()).ok();
                    }
                }
            }
            Err(e) => {
                if num_uart_errors % 1000 == 0 {
                    error!("Error reading TX{} uart: {}", tx_ord + 1, e);
                }
                parser.reset();
                num_uart_errors += 1;
            }
        }
    }
}

#[embassy_executor::task]
async fn write_data_to_txs(
    mut tx1_uart: UartTx<'static, USART2, DMA1_CH3>,
    mut tx2_uart: UartTx<'static, USART3, DMA1_CH5>,
) {
    loop {
        let packet = HANDSET_TO_TX_PACKETS_QUEUE.receive().await;
        let (tx1_res, tx2_res) = join(
            tx1_uart.write(packet.as_slice()),
            tx2_uart.write(packet.as_slice()),
        ).await;
        if let Err(e) = tx1_res {
            error!("Cannot write data to TX1: {}", e);
        }
        if let Err(e) = tx2_res {
            error!("Cannot write data to TX2: {}", e);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(mcu_config());
    info!("Starting ELRS real diversity (handset side)");

    bind_interrupts!(struct Irqs {
        USART1 => InterruptHandler<USART1>;
        USART2 => InterruptHandler<USART2>;
        USART3 => InterruptHandler<USART3>;
    });
    let crsf_uart_config = {
        let mut cfg = UartConfig::default();
        cfg.baudrate = CRSF_BAUDRATE;
        cfg.invert_rx = true;
        cfg.invert_tx = true;
        // Do we need it?
        // cfg.detect_previous_overrun = true;
        cfg
    };

    let handset_uart = Uart::new_half_duplex_on_rx(
        p.USART1, p.PA10, Irqs, p.DMA1_CH1, p.DMA1_CH2, crsf_uart_config
    ).unwrap();
    let (mut handset_uart_out, handset_uart_in) = handset_uart.split();
    let tx1_uart = Uart::new_half_duplex_on_rx(
        p.USART2, p.PA3, Irqs, p.DMA1_CH3, p.DMA1_CH4, crsf_uart_config
    ).unwrap();
    let (tx1_uart_out, tx1_uart_in) = tx1_uart.split();
    let tx2_uart = Uart::new_half_duplex_on_rx(
        p.USART3, p.PB11, Irqs, p.DMA1_CH5, p.DMA1_CH6, crsf_uart_config
    ).unwrap();
    let (tx2_uart_out, tx2_uart_in) = tx2_uart.split();

    spawner.spawn(
        blink(p.PC6.degrade())
    ).unwrap();
    spawner.spawn(
        read_handset_data(handset_uart_in),
    ).unwrap();
    spawner.spawn(
        read_tx1_data(tx1_uart_in),
    ).unwrap();
    spawner.spawn(
        read_tx2_data(tx2_uart_in),
    ).unwrap();
    spawner.spawn(
        write_data_to_txs(tx1_uart_out, tx2_uart_out),
    ).unwrap();

    loop {
        let packet = TX_TO_HANDSET_PACKETS_QUEUE.receive().await;
        match handset_uart_out.write(packet.as_slice()).await {
            Ok(()) => {}
            Err(e) => {
                error!("Erro writing data to handset uart: {}", e);
            }
        }
    }
}
