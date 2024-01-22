#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_stm32::Config;
use embassy_time::Timer;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let config = Config::default();
    let _p = embassy_stm32::init(config);

    loop {
        defmt::info!("Hello World!");
        Timer::after_secs(1).await;
    }
}
