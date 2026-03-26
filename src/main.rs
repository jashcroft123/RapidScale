#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Input;
use embassy_rp::i2c::{self, Config};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Timer};
use panic_probe as _;
use variegated_nau7802::{Gain, Ldo, Nau7802, Nau7802DataAvailableStrategy, SamplesPerSecond};

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

type ADCChannel = Channel<CriticalSectionRawMutex, (i32, Duration), 64>;
static CHANNEL: ADCChannel = Channel::new();

#[embassy_executor::task]
async fn sampler_task(i2c: i2c::I2c<'static, I2C0, i2c::Async>, drdy_pin: Input<'static>) {
    let delay = Delay;
    let strategy = Nau7802DataAvailableStrategy::DrdyPin(drdy_pin);
    let mut nau = Nau7802::new(i2c, strategy, delay, Some(0x2A));

    if let Err(e) = nau
        .init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS320)
        .await
    {
        log::error!("Sampler: Failed to initialize NAU7802: {:?}", e);
        return;
    }

    log::info!("Sampler: NAU7802 initialized at 320 SPS");

    loop {
        let start = Instant::now();
        // Wait for data to be available using the DRDY pin
        if let Ok(()) = nau.wait_for_data_available().await {
            // Read the measurement
            if let Ok(reading) = nau.read().await {
                let end = Instant::now();
                let duration = end - start;
                // Send to channel (non-blocking)
                let _ = CHANNEL.try_send((reading, duration));
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(driver)).unwrap();

    Timer::after_millis(2000).await; // wait for USB to enumerate

    log::info!("Starting...");

    // // // Configure I2C
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c_config = Config::default();
    i2c_config.frequency = 800_000;
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

    // // Initialize DRDY pin
    let drdy_pin = embassy_rp::gpio::Input::new(p.PIN_6, embassy_rp::gpio::Pull::Up);

    // Spawn sampler task
    spawner.spawn(sampler_task(i2c, drdy_pin)).unwrap();

    let mut last_report_time = Instant::now();
    let mut sample_count = 0u32;
    let mut total_duration = Duration::from_ticks(0);

    log::info!("Starting stats consumer...");

    loop {
        // Receive data from the sampler task
        let (_reading, duration) = CHANNEL.receive().await;

        sample_count += 1;
        total_duration += duration;

        if Instant::now() - last_report_time >= Duration::from_secs(1) {
            if sample_count > 0 {
                let avg_ms = total_duration.as_millis() as f32 / sample_count as f32;
                log::info!("--- Stats (last second) ---");
                log::info!("Samples: {}", sample_count);
                log::info!("Avg Duration: {} ms", avg_ms);
                log::info!("---------------------------");
            }
            sample_count = 0;
            total_duration = Duration::from_ticks(0);
            last_report_time = Instant::now();
        }
    }
}
