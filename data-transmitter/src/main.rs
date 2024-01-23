#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_stm32::{
    dac::{DacChannel, Value}, peripherals::{DAC, DMA1_CH5}, rcc::{AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllSource, Sysclk}, time::hz, Config
};
use embassy_time::{Ticker, Timer};
use panic_probe as _;

static TRANSMISSION: &[u8] = include_bytes!("../transmission.bin");

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

    let mut dac = embassy_stm32::dac::DacChannel::new(dp.DAC, dp.DMA1_CH5, dp.PA4);

    dac.set(Value::Bit8(0));
    dac.enable();

    loop {
        defmt::info!("Transmitting data now");
        transmit_data(
            &mut dac,
            &[0, 1],
            2400,
        )
        .await;
        Timer::after_secs(1).await;
    }
}

const BIT_PATTERNS: [Value; 5] = [
    Value::Bit8(50),
    Value::Bit8(100),
    Value::Bit8(150),
    Value::Bit8(200),
    Value::Bit8(250),
];

async fn transmit_data(
    dac: &mut DacChannel<'static, DAC, 1, DMA1_CH5>,
    data: &[u8],
    data_rate: u64,
) {
    let mut previous_state = None;
    let mut ticker = Ticker::every(embassy_time::Duration::from_hz(data_rate * 4));

    for byte in data {
        for i in (0..8u8).step_by(2) {
            let bits = (byte & (0x03 << i)) >> i;

            match previous_state.as_mut() {
                Some(previous_state) => {
                    let next_state = (*previous_state + bits + 1) % 5;
                    *previous_state = next_state;
                    dac.set(BIT_PATTERNS[next_state as usize]);
                }
                None => {
                    previous_state = Some(bits);
                    dac.set(BIT_PATTERNS[bits as usize]);
                }
            };

            ticker.next().await;
        }
    }

    dac.set(Value::Bit8(0));
}
