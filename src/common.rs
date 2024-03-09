use embassy_stm32::Config;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH2, DMA1_CH4, DMA1_CH6, USART1, USART2, USART3};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Error as UartError, UartRx};
use embassy_time::Timer;

// AUX 4 is an SD switch on Radiomaster by default
pub static AUX_SWITCH: u8 = u8_from_option_str(core::option_env!("ELRS_REAL_DIVERSITY_AUX_SWITCH"), 4);
static CONTROL_CHANNELS: u8 = 4;
pub static SWITCH_CHANNEL_IX: usize = (CONTROL_CHANNELS + AUX_SWITCH - 1) as usize;
pub static TRIGGER_VAL: u16 = 900;

const fn u8_from_option_str(s: Option<&'static str>, default: u8) -> u8 {
    match s {
        Some(s) => {
            let mut bytes = s.as_bytes();
            let mut val = 0;
            while let [byte, rest @ ..] = bytes {
                core::assert!(b'0' <= *byte && *byte <= b'9', "invalid digit");
                val = val * 10 + (*byte - b'0') as u8;
                bytes = rest;
            }
            val
        }
        None => default,
    }
}

#[allow(unused)]
pub(crate) enum AnyUartRx {
    Uart1 { uart: UartRx<'static, USART1, DMA1_CH2> },
    Uart2 { uart: UartRx<'static, USART2, DMA1_CH4> },
    Uart3 { uart: UartRx<'static, USART3, DMA1_CH6> },
}

#[allow(unused)]
impl AnyUartRx {
    pub(crate) async fn read(&mut self, buf: &mut [u8]) -> Result<(), UartError> {
        use AnyUartRx::*;

        match self {
            Uart1 { uart } => uart.read(buf).await,
            Uart2 { uart } => uart.read(buf).await,
            Uart3 { uart } => uart.read(buf).await,
        }
    }

    pub(crate) async fn read_until_idle(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        use AnyUartRx::*;

        match self {
            Uart1 { uart } => uart.read_until_idle(buf).await,
            Uart2 { uart } => uart.read_until_idle(buf).await,
            Uart3 { uart } => uart.read_until_idle(buf).await,
        }
    }
}

pub(crate) fn mcu_config() -> Config {
    use embassy_stm32::rcc::*;

    let mut cfg = embassy_stm32::Config::default();
    // Set 160MHz system clock frequency, 8 * 40 / 2
    cfg.rcc = embassy_stm32::rcc::Config {
        mux: ClockSrc::PLL,
        pll: Some(Pll {
            source: PllSource::HSE(Hertz::mhz(8)),
            prediv_m: PllM::DIV1,
            mul_n: PllN::MUL40,
            div_p: None,
            div_q: None,
            div_r: Some(PllR::DIV2),
        }),
        ..Default::default()
    };
    cfg
}

pub(crate) async fn blink(led: AnyPin) {
    let mut led = Output::new(led, Level::High, Speed::Low);

    led.set_low();
    loop {
        // Number of short blinks is equal to number of AUX
        for _ in 0..AUX_SWITCH {
            led.set_high();
            Timer::after_millis(200).await;
            led.set_low();
            Timer::after_millis(200).await;
        }

        Timer::after_millis(2000).await;
    }
}
