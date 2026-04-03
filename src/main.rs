#![no_std]
#![no_main]

mod display;
mod filtering;
mod sampler;
mod stability;
mod types;

use display::{CORE1_EXECUTOR, CORE1_STACK, DISPLAY_WATCH, display_task};
use filtering::{EMA, Filter, FilterStack, Median, SMA};
use sampler::{CHANNEL, sampler_task};
use stability::StabilityDetector;
use types::{DisplayData, ScaleMode, ScaleState};

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::multicore::spawn_core1;
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;

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
    if let Err(e) = spawner.spawn(logger_task(driver)) {
        panic!("Failed to spawn logger task: {:?}", e);
    }
    Timer::after(Duration::from_millis(2000)).await;
    log::info!("Starting scale...");

    // I2C configuration
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, I2cIrqs, i2c_config);

    // Spawn sampler task
    let drdy_pin = Input::new(p.PIN_6, Pull::Up);
    if let Err(e) = spawner.spawn(sampler_task(i2c, drdy_pin)) {
        log::error!("Main: Failed to spawn sampler task: {:?}", e);
    }

    // LCD config
    let mut display_spi_config = SpiConfig::default();
    display_spi_config.frequency = 62_500_000;

    let display_spi = Spi::new_blocking_txonly(p.SPI0, p.PIN_18, p.PIN_19, display_spi_config);
    let dc = Output::new(p.PIN_16, Level::Low);
    let cs = Output::new(p.PIN_17, Level::High);
    let rst = Output::new(p.PIN_20, Level::Low);
    let mut _bl = Output::new(p.PIN_21, Level::High);

    // Launch display task on core 1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor_uninit = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_EXECUTOR) };
            let executor = executor_uninit.write(embassy_executor::Executor::new());
            executor.run(|spawner| {
                if let Err(e) = spawner.spawn(display_task(display_spi, dc, cs, rst)) {
                    log::error!("Core 1: Failed to spawn display task: {:?}", e);
                }
            });
        },
    );

    // Setup filtering
    let mut sma = SMA::<16>::new();
    let mut ema = EMA::<32>::new();
    let mut median = Median::<5>::new();
    let mut filters: [&mut dyn Filter; 3] = [&mut sma, &mut ema, &mut median];
    let mut average_value = FilterStack::new(&mut filters);

    let mut last_value: Option<i32> = None;
    let mut mode = ScaleMode::Stable;
    let mut last_report_time = Instant::now();

    let mut tare_value: SMA<160> = SMA::new();
    let mut tare_offset = 0;
    let mut state = ScaleState::Tare;

    let scale_factor = 0.0002627;
    let mut tare_completed = false;

    // Stability Stack: One decorator (StabilityStack) combines detectors and debounces them
    let mut variance_source = stability::VarianceDetector::<32>::new(50_000_000);
    let mut jump_source = stability::DifferenceDetector::new(20000);
    let mut detectors: [&mut dyn StabilityDetector; 2] = [&mut variance_source, &mut jump_source];
    let mut stability = stability::StabilityStack::new(&mut detectors, 15);

    loop {
        let reading =
            match embassy_time::with_timeout(Duration::from_secs(5), CHANNEL.receive()).await {
                Ok(r) => r,
                Err(_) => {
                    log::warn!(
                        "Main: Still waiting for data from sampler (check I2C/NAU7802 cabling)..."
                    );
                    continue;
                }
            };

        match state {
            ScaleState::Tare => {
                let output = tare_value.add(reading);
                if tare_value.is_saturated() {
                    tare_offset = output;
                    state = ScaleState::Reading;
                    log::info!("Tare completed: offset = {}", tare_offset);
                    tare_completed = true;
                    DISPLAY_WATCH.sender().send(DisplayData {
                        value: 0.0,
                        tare_flag: true,
                    });
                }
            }
            ScaleState::Reading => {
                let is_unstable = stability.is_unstable(reading);

                let new_mode = if is_unstable {
                    ScaleMode::Fast
                } else {
                    ScaleMode::Stable
                };

                if mode == ScaleMode::Stable && new_mode == ScaleMode::Fast {
                    average_value.reset();
                    stability.reset(); // Also reset stability for a clean start
                }
                mode = new_mode;

                let output = match mode {
                    ScaleMode::Fast => reading,
                    ScaleMode::Stable => average_value.add(reading),
                };
                let saturated = mode == ScaleMode::Stable && average_value.is_saturated();

                let output_calibrated: f32 =
                    libm::roundf((output - tare_offset) as f32 * scale_factor * 1000.0) / 1000.0;

                DISPLAY_WATCH.sender().send(DisplayData {
                    value: output_calibrated,
                    tare_flag: tare_completed,
                });

                let now = Instant::now();
                if now - last_report_time >= Duration::from_millis(500) {
                    log::info!(
                        "Mode: {:?}, Value: {:.3}, saturated: {}",
                        mode,
                        output_calibrated,
                        saturated,
                    );
                    last_report_time = now;
                }
            }
        }
    }
}
