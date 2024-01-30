#![no_main]
#![no_std]

use core::sync::atomic::AtomicUsize;

use defmt::unwrap;
use defmt_rtt as _;
use embassy_stm32::{
    dac::{DacChannel, Value},
    exti::ExtiInput,
    gpio::Input,
    peripherals::{DAC, DMA1_CH5, PC13},
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllSource,
        Sysclk,
    },
    time::hz,
    Config,
};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;

mod bad_apple;

const DATA_RATES: [u64; 5] = [1000, 3000, 6000, 16000, 32000];
static SELECTED_DATA_RATE: AtomicUsize = AtomicUsize::new(2);

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
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

    spawner.must_spawn(button_watcher(ExtiInput::new(
        Input::new(dp.PC13, embassy_stm32::gpio::Pull::None),
        dp.EXTI13,
    )));

    loop {
        for frame in bad_apple::BAD_APPLE_SEQUENCE {
            assert_eq!(&frame[0..2], b"BM");
            transmit_data(
                &mut dac,
                frame,
                DATA_RATES[SELECTED_DATA_RATE.load(core::sync::atomic::Ordering::SeqCst)],
            )
            .await;
            Timer::after_millis(10).await; // Give time for processing on the receiver
        }
    }
}

#[embassy_executor::task]
async fn button_watcher(mut button: ExtiInput<'static, PC13>) {
    loop {
        button.wait_for_falling_edge().await;

        unwrap!(SELECTED_DATA_RATE.fetch_update(
            core::sync::atomic::Ordering::SeqCst,
            core::sync::atomic::Ordering::SeqCst,
            |x| Some((x + 1) % DATA_RATES.len()),
        ));

        defmt::info!(
            "New datarate: {}",
            DATA_RATES[SELECTED_DATA_RATE.load(core::sync::atomic::Ordering::SeqCst)]
        );

        // Stop bouncing
        Timer::after(Duration::from_millis(200)).await;
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
    defmt::info!("Transmitting data now");

    let mut previous_state = None;
    let mut ticker = Ticker::every(embassy_time::Duration::from_hz(data_rate * 4));

    for byte in data {
        for i in (0..8u8).step_by(2).rev() {
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
