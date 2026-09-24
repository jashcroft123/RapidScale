#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;
use variegated_nau7802::{Gain, Ldo, Nau7802, Nau7802DataAvailableStrategy, SamplesPerSecond};

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

bind_interrupts!(struct I2cIrqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // USB logger
    let driver = Driver::new(p.USB, UsbIrqs);
    let _ = spawner.spawn(logger_task(driver));

    // Wait for the logger to settle
    Timer::after(Duration::from_millis(2000)).await;

    // Header for CSV
    log::info!("timestamp_ms,raw_value");

    // I2C configuration for NAU7802
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, I2cIrqs, i2c_config);

    let drdy_pin = Input::new(p.PIN_6, Pull::Up);
    let strategy = Nau7802DataAvailableStrategy::DrdyPin(drdy_pin);
    let mut nau = Nau7802::new(i2c, strategy, embassy_time::Delay, Some(0x2A));

    if let Err(e) = nau
        .init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS320)
        .await
    {
        log::error!("Failed to initialize NAU7802: {:?}", e);
        return;
    }

    let start_time = Instant::now();

    loop {
        if let Ok(()) = nau.wait_for_data_available().await {
            if let Ok(reading) = nau.read().await {
                let elapsed = Instant::now() - start_time;
                // CSV Output format
                log::info!("{},{}", elapsed.as_millis(), reading);
            }
        }
    }
}
