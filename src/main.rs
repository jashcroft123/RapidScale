#![no_std]
#![no_main]

mod display;
mod filtering;
mod sampler;
mod stability;
mod types;

use display::{CORE1_EXECUTOR, CORE1_STACK, DISPLAY_WATCH, display_task};
use filtering::{EMA, Filter, FilterStack, HampelFilter, NotchFilter, SMA, SavitzkyGolay7};
use sampler::{CHANNEL, sampler_task};
use stability::{StabilityDetector, StabilityLevel, StabilitySource};
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

    // ------------------------
    // Continuous Primary 50Hz Notch Filter
    // Strips 50Hz mains power hum upfront from all samples
    // ------------------------
    let mut primary_notch = NotchFilter::new_50hz_320sps();

    // ------------------------
    // Filter Architecture
    // ------------------------

    // FAST MODE STACK: Responsive tracking with minimal latency for live pouring
    let mut fast_ema = EMA::<12>::new();
    let mut fast_filters: [&mut dyn Filter; 1] = [&mut fast_ema];
    let mut fast_stack = FilterStack::new(&mut fast_filters);

    // SETTLING MODE STACK: Moderate smoothing to damp liquid sloshing
    let mut settle_sma = SMA::<32>::new();
    let mut settle_sg = SavitzkyGolay7::new();
    let mut settle_filters: [&mut dyn Filter; 2] = [&mut settle_sma, &mut settle_sg];
    let mut settle_stack = FilterStack::new(&mut settle_filters);

    // STABLE MODE STACK: Precision smoothing with outlier rejection
    let mut stable_hampel = HampelFilter::<7>::new(3.0);
    let mut stable_sma = SMA::<96>::new();
    let mut stable_filters: [&mut dyn Filter; 2] = [&mut stable_hampel, &mut stable_sma];
    let mut stable_stack = FilterStack::new(&mut stable_filters);

    // ------------------------
    // Stability Stack:
    // 1. DifferenceDetector: detects sudden jumps (> 300 counts ~ 0.08g)
    // 2. MedianDecorator<5>: protects difference detector against single-sample EMI spikes
    // 3. VarianceDetector<64>: detects noise/drift over ~200ms window (threshold = 3500 counts^2 ~ 0.015g sigma)
    // ------------------------
    let mut variance_raw = stability::VarianceDetector::<64>::new(3_500);
    let mut jump_raw = stability::DifferenceDetector::new(300);
    let mut jump_med = stability::MedianDecorator::<5>::new(&mut jump_raw);

    let mut detectors: [&mut dyn StabilitySource; 2] = [&mut variance_raw, &mut jump_med];

    // Motion selects Fast. The first quiet sample selects Settling.
    // Stable after 96 quiet samples (~300ms at 320 SPS).
    let mut stability = stability::StabilityStack::new(&mut detectors, 96);

    let mut mode = ScaleMode::Stable;
    let mut last_report_time = Instant::now();

    let mut tare_value: SMA<160> = SMA::new();
    let mut tare_offset = 0;
    let mut last_output = 0;
    let mut state = ScaleState::Tare;

    let scale_factor = 0.0002627;
    let mut tare_completed = false;

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

        // Always run the primary notch filter so it stays in steady state
        let clean = primary_notch.add(reading);

        match state {
            ScaleState::Tare => {
                let output = tare_value.add(clean);
                if tare_value.is_saturated() {
                    tare_offset = output;
                    last_output = output;
                    fast_stack.init_to(output);
                    settle_stack.init_to(output);
                    stable_stack.init_to(output);
                    stability.init_to(output);

                    state = ScaleState::Reading;
                    mode = ScaleMode::Stable;
                    log::info!("Tare completed: offset = {}", tare_offset);
                    tare_completed = true;
                    DISPLAY_WATCH.sender().send(DisplayData {
                        value: 0.0,
                        tare_flag: true,
                        mode: ScaleMode::Stable,
                    });
                }
            }
            ScaleState::Reading => {
                let level = stability.check(clean);

                let next_mode = match level {
                    StabilityLevel::Unstable => ScaleMode::Fast,
                    StabilityLevel::Settling => ScaleMode::Settling,
                    StabilityLevel::Stable => ScaleMode::Stable,
                };

                // HANDLE MODE TRANSITIONS (Bumpless seeding with last filtered output)
                if mode != next_mode {
                    match next_mode {
                        ScaleMode::Fast => fast_stack.init_to(last_output),
                        ScaleMode::Settling => settle_stack.init_to(last_output),
                        ScaleMode::Stable => stable_stack.init_to(last_output),
                    }
                }
                mode = next_mode;

                // ADD TO ACTIVE FILTER
                let output = match mode {
                    ScaleMode::Fast => fast_stack.add(clean),
                    ScaleMode::Settling => settle_stack.add(clean),
                    ScaleMode::Stable => stable_stack.add(clean),
                };
                last_output = output;

                let output_calibrated: f32 =
                    libm::roundf((output - tare_offset) as f32 * scale_factor * 1000.0) / 1000.0;

                DISPLAY_WATCH.sender().send(DisplayData {
                    value: output_calibrated,
                    tare_flag: tare_completed,
                    mode,
                });

                let now = Instant::now();
                if now - last_report_time >= Duration::from_millis(500) {
                    log::info!(
                        "Mode: {:?}, Stability: {:?}, Value: {:.3} g",
                        mode,
                        level,
                        output_calibrated,
                    );
                    last_report_time = now;
                }
            }
        }
    }
}
