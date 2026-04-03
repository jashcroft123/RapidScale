#![no_std]
#![no_main]

mod display;
mod filtering;
mod sampler;
mod stability;
mod types;

use display::{CORE1_EXECUTOR, CORE1_STACK, DISPLAY_WATCH, display_task};
use filtering::{
    EMA, Filter, FilterStack, HampelFilter, KalmanFilter, Median, NotchFilter, SMA, SavitzkyGolay7,
};
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
    // Filter Architecture
    // ------------------------

    // FAST MODE STACK (Super damped for molasses-like movement)
    let mut fast_notch = NotchFilter::new_50hz_320sps();
    let mut fast_ema = EMA::<64>::new();
    let mut fast_filters: [&mut dyn Filter; 2] = [&mut fast_notch, &mut fast_ema];
    let mut fast_stack = FilterStack::new(&mut fast_filters);

    // SETTLING MODE STACK (Slow, deliberate approach)
    let mut settle_notch = NotchFilter::new_50hz_320sps();
    let mut settle_sma = SMA::<160>::new();
    let mut settle_sg = SavitzkyGolay7::new();
    let mut settle_filters: [&mut dyn Filter; 3] =
        [&mut settle_notch, &mut settle_sma, &mut settle_sg];
    let mut settle_stack = FilterStack::new(&mut settle_filters);

    // STABLE MODE STACK (Ultra-Precise Lab Grade - extremely slow)
    let mut stable_hampel = HampelFilter::<7>::new(3.0);
    let mut stable_notch = NotchFilter::new_50hz_320sps();
    let mut stable_sma = SMA::<640>::new();
    let mut stable_sg = SavitzkyGolay7::new();
    let mut stable_kalman = KalmanFilter::new(0.000001, 100.0);
    let mut stable_filters: [&mut dyn Filter; 5] = [
        &mut stable_hampel,
        &mut stable_notch,
        &mut stable_sma,
        &mut stable_sg,
        &mut stable_kalman,
    ];
    let mut stable_stack = FilterStack::new(&mut stable_filters);

    // Stability Stack: (Super damped thresholds and long debouncing)
    // 1. MedianDecorator kills impulse noise (tapping).
    // 2. DeadbandDecorator kills continuous vibration/ripple.
    let mut variance_raw = stability::VarianceDetector::<128>::new(30_000_000); 
    let mut jump_raw = stability::DifferenceDetector::new(40000); // 40k counts threshold for jumps
    
    let mut variance_med = stability::MedianDecorator::<5>::new(&mut variance_raw);
    let mut jump_med = stability::MedianDecorator::<5>::new(&mut jump_raw);

    let mut variance_source = stability::DeadbandDecorator::new(&mut variance_med, 15000);
    let mut jump_source = stability::DeadbandDecorator::new(&mut jump_med, 15000);
    
    let mut detectors: [&mut dyn StabilitySource; 2] = [&mut variance_source, &mut jump_source];
    
    // Settling = 64 samples (~200ms), Stable = 160 samples (~500ms)
    let mut stability = stability::StabilityStack::new(&mut detectors, 64, 160);

    let mut mode = ScaleMode::Fast;
    let mut last_report_time = Instant::now();

    let mut tare_value: SMA<160> = SMA::new();
    let mut tare_offset = 0;
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
                let level = stability.check(reading);

                let next_mode = match level {
                    StabilityLevel::Unstable => ScaleMode::Fast,
                    StabilityLevel::Settling => ScaleMode::Settling,
                    StabilityLevel::Stable => ScaleMode::Stable,
                };

                // HANDLE MODE TRANSITIONS (Seeding)
                if mode != next_mode {
                    match next_mode {
                        ScaleMode::Fast => {
                            fast_stack.init_to(reading);
                            if mode == ScaleMode::Stable {
                                stability.reset();
                            }
                        }
                        ScaleMode::Settling => settle_stack.init_to(reading),
                        ScaleMode::Stable => stable_stack.init_to(reading),
                    }
                }
                mode = next_mode;

                // ADD TO FILTER
                let output = match mode {
                    ScaleMode::Fast => fast_stack.add(reading),
                    ScaleMode::Settling => settle_stack.add(reading),
                    ScaleMode::Stable => stable_stack.add(reading),
                };

                let output_calibrated: f32 =
                    libm::roundf((output - tare_offset) as f32 * scale_factor * 1000.0) / 1000.0;

                DISPLAY_WATCH.sender().send(DisplayData {
                    value: output_calibrated,
                    tare_flag: tare_completed,
                });

                let now = Instant::now();
                if now - last_report_time >= Duration::from_millis(500) {
                    log::info!(
                        "Mode: {:?}, Stability: {:?}, Value: {:.3}",
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
