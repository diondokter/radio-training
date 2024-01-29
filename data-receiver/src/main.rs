#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use crate::adc::Adc;
use defmt_rtt as _;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::interrupt::{self, InterruptExt};
use embassy_stm32::peripherals::{PB14, PB7};
use embassy_stm32::time::mhz;
use embassy_stm32::{
    gpio::Output,
    peripherals::ADC1,
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllSource,
        Sysclk,
    },
    time::hz,
    Config,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_graphics::{image::Image, prelude::*};
use heapless::Vec;
use panic_probe as _;
use shared::DATA_RATE;
use ssd1306::{prelude::*, Ssd1306};
use static_cell::{make_static, StaticCell};
use tinybmp::Bmp;

mod adc;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::interrupt]
unsafe fn PVD() {
    EXECUTOR_HIGH.on_interrupt()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = Config::default();

    config.rcc.hse = Some(Hse {
        freq: hz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.hsi = false;
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL180,
        divp: Some(PllPDiv::DIV2),
        divq: None,
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;

    let dp = embassy_stm32::init(config);

    defmt::info!("Booted up");

    let mut adc = Adc::new(dp.ADC1, &mut embassy_time::Delay);
    let mut adc_pin = dp.PA5;

    adc.set_resolution(embassy_stm32::adc::Resolution::EightBit);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::Cycles3);
    adc.set_running(&mut adc_pin);

    let mut display_spi_config = embassy_stm32::spi::Config::default();
    display_spi_config.frequency = mhz(10);
    let display_spi = embassy_stm32::spi::Spi::new_txonly(
        dp.SPI1,
        dp.PB3,
        dp.PB5,
        dp.DMA1_CH1,
        dp.DMA1_CH2,
        display_spi_config,
    );

    let mut display = Ssd1306::new(
        SPIInterface::new(
            display_spi,
            Output::new(
                dp.PC7,
                embassy_stm32::gpio::Level::Low,
                embassy_stm32::gpio::Speed::VeryHigh,
            ),
            Output::new(
                dp.PA4,
                embassy_stm32::gpio::Level::Low,
                embassy_stm32::gpio::Speed::VeryHigh,
            ),
        ),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display.init().unwrap();
    display.clear_buffer();
    display.flush().unwrap();

    let error_led = Output::new(
        dp.PB14,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );
    let wait_led = Output::new(
        dp.PB7,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    let next_frame = make_static!(embassy_sync::signal::Signal::new());
    next_frame.reset();

    interrupt::PVD.set_priority(interrupt::Priority::P15);
    let high_prio_spawner = EXECUTOR_HIGH.start(interrupt::PVD);
    high_prio_spawner.must_spawn(data_receiver(adc, next_frame, wait_led));

    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(display_drawer(display, next_frame, error_led));
    });
}

async fn receive_data<const N: usize>(adc: &mut Adc<'_, ADC1>) -> heapless::Vec<u8, N> {
    let mut buffer = heapless::Vec::new();

    let mut previous_state = None;

    defmt::info!("Waiting for start of transmission");
    while (adc.read() as u8) < 25 {
    }

    for _ in 0..N {
        let mut byte = 0;

        for i in (0..8).step_by(2) {
            Timer::after(Duration::from_hz(DATA_RATE * 4 * 4)).await;

            let mut previous_new_state = None;

            let new_state = loop {
                let new_state = loop {
                    let mut adc_value = adc.read() as u8;

                    let adc_value = loop {
                        let next_adc_value = adc.read() as u8;
                        if adc_value.abs_diff(next_adc_value) < 20 {
                            break next_adc_value;
                        } else {
                            adc_value = next_adc_value;
                        }
                    };

                    let read_state = if adc_value < 25 {
                        return buffer;
                    } else if adc_value > 40 && adc_value < 60 {
                        0
                    } else if adc_value > 90 && adc_value < 110 {
                        1
                    } else if adc_value > 140 && adc_value < 160 {
                        2
                    } else if adc_value > 190 && adc_value < 210 {
                        3
                    } else if adc_value > 240 {
                        4
                    } else {
                        continue;
                    };

                    if let Some(previous_state) = previous_state {
                        if read_state != previous_state {
                            break read_state;
                        }
                    } else {
                        break read_state;
                    }
                };

                match previous_new_state {
                    Some(previous_new_state) if new_state == previous_new_state => break new_state,
                    _ => previous_new_state = Some(new_state),
                }
            };

            let bits = match previous_state {
                Some(previous_state) => {
                    let change_amount = if new_state > previous_state {
                        new_state - previous_state
                    } else {
                        new_state + 5 - previous_state
                    };

                    change_amount - 1
                }
                None => new_state,
            };

            previous_state = Some(new_state);

            byte |= bits << i
        }

        buffer.push(byte).unwrap();
    }

    buffer
}

type Display = Ssd1306<
    ssd1306::prelude::SPIInterface<
        embassy_stm32::spi::Spi<
            'static,
            embassy_stm32::peripherals::SPI1,
            embassy_stm32::peripherals::DMA1_CH1,
            embassy_stm32::peripherals::DMA1_CH2,
        >,
        Output<'static, embassy_stm32::peripherals::PC7>,
        Output<'static, embassy_stm32::peripherals::PA4>,
    >,
    ssd1306::prelude::DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
>;

#[embassy_executor::task]
async fn data_receiver(
    mut adc: Adc<'static, ADC1>,
    next_frame: &'static Signal<CriticalSectionRawMutex, Vec<u8, 898>>,
    mut wait_led: Output<'static, PB7>,
) {
    loop {
        next_frame.signal(receive_data(&mut adc).await);

        wait_led.set_high();
        Timer::after_millis(shared::MILLIS_BETWEEN_TRANSMISSIONS.saturating_sub(2)).await;
        wait_led.set_low();
    }
}

#[embassy_executor::task]
async fn display_drawer(
    mut display: Display,
    next_frame: &'static Signal<CriticalSectionRawMutex, Vec<u8, 898>>,
    mut error_led: Output<'static, PB14>,
) {
    loop {
        let next_frame = next_frame.wait().await;

        defmt::info!("Got frame with len: {}", next_frame.len());

        match Bmp::from_slice(&next_frame) {
            Ok(raw_image) => {
                error_led.set_low();
                display.clear_buffer();
                let image = Image::new(&raw_image, Point::zero());
                image.draw(&mut display).unwrap();
            }
            Err(e) => {
                error_led.set_high();
                defmt::error!("{}", defmt::Debug2Format(&e));
            }
        };

        display.flush().unwrap();
    }
}
