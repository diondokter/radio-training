#![no_main]
#![no_std]

use crate::adc::Adc;
use defmt_rtt as _;
use embassy_stm32::{
    peripherals::ADC1,
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllSource,
        Sysclk,
    },
    time::hz,
    Config,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;

const DATA_RATE: u64 = 2400;

mod adc;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
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

    let mut adc = Adc::new(dp.ADC1, &mut embassy_time::Delay);
    let mut adc_pin = dp.PA5;

    adc.set_resolution(embassy_stm32::adc::Resolution::EightBit);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::Cycles3);
    adc.set_running(&mut adc_pin);

    loop {
        let mut buffer = [0; 1024];

        let len = receive_data(&mut adc, &mut buffer).await;

        defmt::info!("{}", buffer[..len.min(1024)]);
    }
}

async fn receive_data(adc: &mut Adc<'_, ADC1>, buffer: &mut [u8]) -> usize {
    buffer.fill(0);

    let mut previous_state = None;

    defmt::info!("Waiting for start of transmission");
    while (adc.read() as u8) < 25 {
        Timer::after(Duration::from_hz(DATA_RATE * 4 * 4)).await;
    }

    for (byte_index, byte) in buffer.iter_mut().enumerate() {
        for i in (0..8).step_by(2) {
            Timer::after(Duration::from_hz(DATA_RATE * 4 * 4)).await;

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
                    return byte_index;
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

            defmt::trace!("{}, {}, {}", previous_state, new_state, bits);

            previous_state = Some(new_state);

            *byte |= bits << i
        }
    }

    buffer.len()
}
