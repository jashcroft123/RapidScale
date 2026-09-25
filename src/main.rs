#![no_std]
#![no_main]

mod device_properties;
mod display;
mod filtering;
mod sampler;
mod stability;
mod types;
mod usb_commands;

use device_properties::{DeviceProperties, PersistentProperties, assign_property};
use display::{CORE1_EXECUTOR, CORE1_STACK, DISPLAY_WATCH, display_task};
use filtering::{EMA, Filter, FilterStack, HampelFilter, NotchFilter, SMA, SavitzkyGolay7};
use sampler::{CHANNEL, sampler_task};
use stability::{StabilityDetector, StabilityLevel, StabilitySource};
use types::{DisplayData, ScaleMode, ScaleState};
use usb_commands::{CommandReceiver, USB_COMMANDS, command_parser_task};

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::multicore::spawn_core1;
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_logger::ReceiverHandler;
use panic_probe as _;

struct Characterisation {
    id: u32,
    known_mass_g: f32,
    cycles: u8,
    convergence_mg: i64,
    window_samples: u16,
    step: u8,
    target_loaded: bool,
    prompted_at: Instant,
    detected_at: Option<Instant>,
    window_count: u16,
    window_sum_mg: i64,
    window_sum_sq_mg: i128,
    previous_window_mean_mg: Option<i64>,
    stable_comparisons: u8,
    background_sum_mg: i64,
    background_sum_sq_mg: i128,
    background_samples: u32,
    loaded_sum_mg: i64,
    loaded_sum_sq_mg: i128,
    loaded_samples: u32,
    settle_sum_ms: u64,
    settle_max_ms: u64,
    settle_count: u8,
}

impl Characterisation {
    fn new(
        id: u32,
        known_mass_g: f32,
        cycles: u8,
        convergence_g: f32,
        window_samples: u16,
    ) -> Self {
        Self {
            id,
            known_mass_g,
            cycles,
            convergence_mg: libm::roundf(convergence_g * 1000.0) as i64,
            window_samples,
            step: 0,
            target_loaded: false,
            prompted_at: Instant::now(),
            detected_at: None,
            window_count: 0,
            window_sum_mg: 0,
            window_sum_sq_mg: 0,
            previous_window_mean_mg: None,
            stable_comparisons: 0,
            background_sum_mg: 0,
            background_sum_sq_mg: 0,
            background_samples: 0,
            loaded_sum_mg: 0,
            loaded_sum_sq_mg: 0,
            loaded_samples: 0,
            settle_sum_ms: 0,
            settle_max_ms: 0,
            settle_count: 0,
        }
    }

    fn reset_window(&mut self) {
        self.window_count = 0;
        self.window_sum_mg = 0;
        self.window_sum_sq_mg = 0;
        self.previous_window_mean_mg = None;
        self.stable_comparisons = 0;
    }
}

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

bind_interrupts!(struct I2cIrqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(4096, log::LevelFilter::Info, driver, CommandReceiver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut flash = Flash::<_, Blocking, { 2 * 1024 * 1024 }>::new_blocking(p.FLASH);
    let mut persistent = PersistentProperties::load(&mut flash);

    // USB logger
    let driver = Driver::new(p.USB, UsbIrqs);
    if let Err(e) = spawner.spawn(logger_task(driver)) {
        panic!("Failed to spawn logger task: {:?}", e);
    }
    if let Err(e) = spawner.spawn(command_parser_task()) {
        panic!("Failed to spawn USB command parser: {:?}", e);
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
    // Two-second averaging in Stable mode dampens stationary ADC noise while
    // Fast and Settling retain their shorter response times.
    let mut stable_sma = SMA::<640>::new();
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
    // Require the same two-second quiet interval as the Stable-mode average.
    let mut stability = stability::StabilityStack::new(&mut detectors, 640);

    let mut mode = ScaleMode::Stable;
    let mut last_report_time = Instant::now();
    let mut last_ui_update_time = Instant::now();

    let mut tare_value: SMA<160> = SMA::new();
    let mut tare_offset = persistent.values.tare_offset_counts;
    let mut last_output = 0;
    let mut state = ScaleState::Tare;

    let mut scale_factor = persistent.values.scale_factor_g_per_count;
    let mut tare_completed = false;
    let mut tare_command_id: Option<u32> = None;
    let mut calibration: Option<(f32, u32, i64, u32)> = None;
    let mut characterisation: Option<Characterisation> = None;
    let mut reading_sequence = 0u32;

    loop {
        while let Ok(command) = USB_COMMANDS.try_receive() {
            let mut fields = command.split(',');
            let wire_protocol = fields.next() == Some("@SCALE/2");
            let (command_id, verb, argument, second_argument) = if wire_protocol {
                let command_kind = fields.next();
                let id = fields.next().and_then(|value| value.parse::<u32>().ok());
                let verb = fields.next();
                let argument = fields.next();
                let second_argument = fields.next();
                if command_kind != Some("CMD")
                    || id.is_none()
                    || verb.is_none()
                    || fields.next().is_some()
                {
                    log::info!("@SCALE/2,ERROR,0,BAD_REQUEST");
                    continue;
                }
                (id.unwrap(), verb.unwrap(), argument, second_argument)
            } else {
                let mut parts = command.split_ascii_whitespace();
                (0, parts.next().unwrap_or(""), parts.next(), parts.next())
            };
            if command_id == 0 && wire_protocol {
                log::info!("@SCALE/2,ERROR,0,BAD_REQUEST");
                continue;
            }
            let invalid_wire_shape = (verb.eq_ignore_ascii_case("HELLO")
                    || verb.eq_ignore_ascii_case("TARE")
                    || verb.eq_ignore_ascii_case("HELP")
                    || verb.eq_ignore_ascii_case("GET"))
                    && (argument.is_some() || second_argument.is_some())
                || ((verb.eq_ignore_ascii_case("CAL")
                        || verb.eq_ignore_ascii_case("CHARACTERISE")
                        || verb.eq_ignore_ascii_case("CHARACTERIZE"))
                        && (argument.is_none() || second_argument.is_some()))
                    || (verb.eq_ignore_ascii_case("SET")
                        && (argument.is_none() || second_argument.is_none()))
                    || (!verb.eq_ignore_ascii_case("SET")
                        && !(verb.eq_ignore_ascii_case("HELLO")
                            || verb.eq_ignore_ascii_case("TARE")
                            || verb.eq_ignore_ascii_case("HELP")
                            || verb.eq_ignore_ascii_case("GET")
                            || verb.eq_ignore_ascii_case("CAL")
                            || verb.eq_ignore_ascii_case("CHARACTERISE")
                            || verb.eq_ignore_ascii_case("CHARACTERIZE"))
                        && (argument.is_some() || second_argument.is_some()));
            if wire_protocol && invalid_wire_shape {
                log::info!("@SCALE/2,ERROR,{},BAD_ARGUMENT", command_id);
                continue;
            }
            if wire_protocol
                && (verb.eq_ignore_ascii_case("CAL")
                    || verb.eq_ignore_ascii_case("CHARACTERISE")
                    || verb.eq_ignore_ascii_case("CHARACTERIZE"))
                && (calibration.is_some() || characterisation.is_some())
            {
                log::info!("@SCALE/2,ERROR,{},BUSY", command_id);
                continue;
            }
            if verb.eq_ignore_ascii_case("HELLO") && wire_protocol && argument.is_none() {
                log::info!("@SCALE/2,HELLO,{},2,RP2350,320", command_id);
            } else if verb.eq_ignore_ascii_case("GET") && wire_protocol {
                send_properties(command_id, &persistent.values);
            } else if verb.eq_ignore_ascii_case("HELP") {
                log::info!(
                    "USB commands: GET | SET,<property>,<value> | TARE | CAL <known_mass_g> | CHARACTERISE <known_mass_g> | HELP. Properties are stored in flash."
                );
                if wire_protocol {
                    log::info!("@SCALE/2,ACK,{},HELP", command_id);
                }
            } else if verb.eq_ignore_ascii_case("SET") && wire_protocol {
                let key = argument.unwrap_or("");
                let value = second_argument.unwrap_or("");
                let before = persistent.values;
                match assign_property(&mut persistent.values, key, value) {
                    Ok(()) => match persistent.save(&mut flash) {
                        Ok(()) => {
                            scale_factor = persistent.values.scale_factor_g_per_count;
                            tare_offset = persistent.values.tare_offset_counts;
                            stability.set_noise_thresholds(
                                persistent.values.tare_noise_variance_counts2,
                            );
                            log::info!("@SCALE/2,PROPERTY_SET,{},{},{}", command_id, key, value);
                        }
                        Err(()) => {
                            persistent.values = before;
                            log::info!(
                                "@SCALE/2,PROPERTY_ERROR,{},{},FLASH_ERROR",
                                command_id,
                                key
                            );
                        }
                    },
                    Err(code) => {
                        log::info!("@SCALE/2,PROPERTY_ERROR,{},{},{}", command_id, key, code)
                    }
                }
            } else if verb.eq_ignore_ascii_case("TARE") {
                if let Some((_, _, _, old_id)) = calibration.take() {
                    if old_id != 0 {
                        log::info!("@SCALE/2,EVENT,CANCELLED,{},TARE", old_id);
                    }
                }
                if let Some(old) = characterisation.take() {
                    if old.id != 0 {
                        log::info!("@SCALE/2,EVENT,CANCELLED,{},TARE", old.id);
                    }
                }
                if let Some(old_id) = tare_command_id.take() {
                    log::info!("@SCALE/2,EVENT,CANCELLED,{},TARE", old_id);
                }
                Filter::reset(&mut tare_value);
                stability.init_to(last_output);
                state = ScaleState::Tare;
                tare_command_id = if wire_protocol {
                    Some(command_id)
                } else {
                    None
                };
                log::info!(
                    "Tare started; remove all load and keep the platform still for 0.5 seconds."
                );
                if wire_protocol {
                    log::info!("@SCALE/2,ACK,{},TARE", command_id);
                }
            } else if verb.eq_ignore_ascii_case("CAL") {
                if state != ScaleState::Reading {
                    log::warn!("CAL unavailable while tare is running.");
                    if wire_protocol {
                        log::info!("@SCALE/2,ERROR,{},NOT_TARED", command_id);
                    }
                    continue;
                }
                let mass = argument.and_then(|s| s.parse::<f32>().ok());
                match mass {
                    Some(mass) if mass.is_finite() && mass > 0.0 => {
                        calibration = Some((mass, 0, 0, command_id));
                        log::info!(
                            "Calibration started for {:.3} g. Keep the known mass already on the platform still.",
                            mass
                        );
                        if wire_protocol {
                            log::info!("@SCALE/2,ACK,{},CAL", command_id);
                        }
                    }
                    _ => {
                        log::warn!("Usage: CAL <known_mass_g> (mass must be positive).");
                        if wire_protocol {
                            log::info!("@SCALE/2,ERROR,{},BAD_ARGUMENT", command_id);
                        }
                    }
                }
            } else if verb.eq_ignore_ascii_case("CHARACTERISE")
                || verb.eq_ignore_ascii_case("CHARACTERIZE")
            {
                let known_mass = argument.and_then(|value| value.parse::<f32>().ok());
                let expected_counts = known_mass
                    .filter(|mass| mass.is_finite() && *mass > 0.0)
                    .map(|mass| (mass / scale_factor).abs());
                if state != ScaleState::Reading
                    || expected_counts.is_none()
                    || !scale_factor.is_finite()
                    || scale_factor == 0.0
                    || expected_counts.is_some_and(|counts| counts < 1000.0)
                {
                    log::warn!(
                        "Usage: CHARACTERISE <known_mass_g>; tare first and use a reference load producing at least 1000 raw counts."
                    );
                    if wire_protocol {
                        log::info!("@SCALE/2,ERROR,{},BAD_ARGUMENT", command_id);
                    }
                } else {
                    let mass = known_mass.unwrap();
                    let before = persistent.values;
                    persistent.values.characterisation_reference_mass_g = mass;
                    persistent.values.characterisation_valid = false;
                    if persistent.save(&mut flash).is_err() {
                        persistent.values = before;
                        if wire_protocol {
                            log::info!("@SCALE/2,ERROR,{},FLASH_ERROR", command_id);
                        }
                        continue;
                    }
                    let run = Characterisation::new(
                        command_id,
                        mass,
                        persistent.values.characterisation_cycles,
                        persistent.values.characterisation_convergence_g,
                        persistent.values.characterisation_window_samples,
                    );
                    characterisation = Some(run);
                    log::info!(
                        "Characterisation started: reference mass {:.3} g; remove it for the empty-platform noise phase.",
                        mass
                    );
                    log::info!("Characterisation step 0: REMOVE_LOAD.");
                    if wire_protocol {
                        log::info!("@SCALE/2,ACK,{},CHARACTERISE", command_id);
                        log::info!("@SCALE/2,EVENT,CHAR_STEP,{},0,REMOVE_LOAD", command_id);
                    }
                }
            } else if !verb.is_empty() {
                log::warn!(
                    "Unknown USB command: {}. Send HELP for commands.",
                    command.as_str()
                );
                if wire_protocol {
                    log::info!("@SCALE/2,ERROR,{},UNKNOWN_COMMAND", command_id);
                }
            }
        }
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
                // Reuse the empty-pan tare interval to measure the scale's
                // stationary noise before choosing the variance threshold.
                let _ = stability.check(clean);
                let output = tare_value.add(clean);
                if tare_value.is_saturated() {
                    tare_offset = output;
                    last_output = output;
                    if let Some((noise_variance, threshold, jump_threshold)) =
                        stability.calibrate_noise_thresholds()
                    {
                        persistent.values.tare_noise_variance_counts2 = noise_variance.max(0);
                        log::info!(
                            "Noise calibration: variance={} counts^2, variance threshold={} counts^2, jump threshold={} counts",
                            noise_variance,
                            threshold,
                            jump_threshold,
                        );
                        if noise_variance.saturating_mul(4) > 100_000 {
                            log::warn!(
                                "Tare noise is high; check that the scale is level, unloaded, and mechanically isolated"
                            );
                        }
                    }
                    persistent.values.tare_offset_counts = tare_offset;
                    persistent.values.tare_valid = true;
                    persistent.values.characterisation_valid = false;
                    let tare_persisted = persistent.save(&mut flash).is_ok();
                    if !tare_persisted {
                        log::error!("Failed to persist tare values to flash.");
                    }
                    fast_stack.init_to(output);
                    settle_stack.init_to(output);
                    stable_stack.init_to(output);
                    stability.init_to(output);

                    state = ScaleState::Reading;
                    mode = ScaleMode::Stable;
                    log::info!("Tare completed: offset = {}", tare_offset);
                    if let Some(command_id) = tare_command_id.take() {
                        if tare_persisted {
                            log::info!("@SCALE/2,EVENT,TARE_DONE,{}", command_id);
                        } else {
                            log::info!("@SCALE/2,ERROR,{},FLASH_ERROR", command_id);
                        }
                    }
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

                if let Some((known_mass, mut count, mut sum, command_id)) = calibration {
                    if level == StabilityLevel::Stable {
                        count += 1;
                        sum += clean as i64;
                        if count >= 640 {
                            let raw_delta = sum as f32 / count as f32 - tare_offset as f32;
                            if raw_delta.abs() < 1.0 {
                                log::error!(
                                    "Calibration failed: measured raw change was too small. Check the load and repeat."
                                );
                                if command_id != 0 {
                                    log::info!("@SCALE/2,ERROR,{},CALIBRATION_FAILED", command_id);
                                }
                            } else {
                                let before = persistent.values;
                                let new_factor = known_mass / raw_delta;
                                if !new_factor.is_finite() || new_factor == 0.0 || new_factor.abs() > 1.0 {
                                    log::error!("Calibration produced an invalid scale factor.");
                                    if command_id != 0 {
                                        log::info!("@SCALE/2,ERROR,{},CALIBRATION_FAILED", command_id);
                                    }
                                    calibration = None;
                                    continue;
                                }
                                persistent.values.scale_factor_g_per_count = new_factor;
                                persistent.values.calibration_last_reference_mass_g = known_mass;
                                persistent.values.calibration_last_raw_delta_counts = raw_delta;
                                persistent.values.characterisation_valid = false;
                                if persistent.save(&mut flash).is_ok() {
                                    scale_factor = new_factor;
                                    log::info!(
                                        "Calibration complete and saved: {:.3} g reference, raw change {:.1} counts, scale factor {:.9} g/count.",
                                        known_mass,
                                        raw_delta,
                                        scale_factor
                                    );
                                    if command_id != 0 {
                                        log::info!(
                                            "@SCALE/2,EVENT,CAL_DONE,{},{:.3},{:.1},{:.9}",
                                            command_id,
                                            known_mass,
                                            raw_delta,
                                            scale_factor
                                        );
                                    }
                                } else {
                                    persistent.values = before;
                                    log::error!(
                                        "Calibration measured but could not be saved to flash."
                                    );
                                    if command_id != 0 {
                                        log::info!("@SCALE/2,ERROR,{},FLASH_ERROR", command_id);
                                    }
                                }
                            }
                            calibration = None;
                        } else {
                            calibration = Some((known_mass, count, sum, command_id));
                        }
                    } else {
                        if count > 0 {
                            log::info!(
                                "Calibration sample interrupted by motion; restarting stable sample."
                            );
                        }
                        calibration = Some((known_mass, 0, 0, command_id));
                    }
                }

                let output_calibrated: f32 =
                    libm::roundf((output - tare_offset) as f32 * scale_factor * 1000.0) / 1000.0;

                if let Some(mut run) = characterisation.take() {
                    let sample_time = Instant::now();
                    let expected_counts = (run.known_mass_g / scale_factor).abs();
                    let raw_delta = (clean as i64 - tare_offset as i64).abs() as f32;
                    let load_detected = raw_delta >= expected_counts * 0.5;
                    let mut keep_run = true;

                    if sample_time.duration_since(run.prompted_at) >= Duration::from_secs(120) {
                        log::warn!(
                            "Characterisation timed out while waiting for step {}.",
                            run.step
                        );
                        if run.id != 0 {
                            log::info!("@SCALE/2,ERROR,{},CHAR_TIMEOUT", run.id);
                        }
                        keep_run = false;
                    } else if load_detected != run.target_loaded {
                        run.detected_at = None;
                        run.reset_window();
                    } else {
                        if run.detected_at.is_none() {
                            run.detected_at = Some(sample_time);
                        }

                        let transition_timed_out = run.detected_at.is_some_and(|detected_at| {
                            sample_time.duration_since(detected_at) >= Duration::from_secs(30)
                        });
                        if transition_timed_out {
                            log::warn!(
                                "Characterisation step {} did not converge within 30 seconds.",
                                run.step
                            );
                            if run.id != 0 {
                                log::info!("@SCALE/2,ERROR,{},CHAR_NOT_STABLE", run.id);
                            }
                            keep_run = false;
                        } else if level != StabilityLevel::Stable {
                            run.reset_window();
                        } else {
                            let mg = libm::roundf(output_calibrated * 1000.0) as i64;
                            run.window_count += 1;
                            run.window_sum_mg += mg;
                            run.window_sum_sq_mg += mg as i128 * mg as i128;

                            if run.window_count >= run.window_samples {
                                let window_mean = run.window_sum_mg / run.window_count as i64;
                                let is_converged =
                                    run.previous_window_mean_mg.is_some_and(|previous| {
                                        (window_mean - previous).abs() <= run.convergence_mg
                                    });
                                run.stable_comparisons = if is_converged {
                                    run.stable_comparisons.saturating_add(1)
                                } else {
                                    0
                                };
                                run.previous_window_mean_mg = Some(window_mean);

                                if run.stable_comparisons >= 1 {
                                    if run.target_loaded {
                                        run.loaded_sum_mg += run.window_sum_mg;
                                        run.loaded_sum_sq_mg += run.window_sum_sq_mg;
                                        run.loaded_samples += run.window_count as u32;
                                    } else {
                                        run.background_sum_mg += run.window_sum_mg;
                                        run.background_sum_sq_mg += run.window_sum_sq_mg;
                                        run.background_samples += run.window_count as u32;
                                    }

                                    if run.step > 0 {
                                        if let Some(detected_at) = run.detected_at {
                                            let settle_ms =
                                                sample_time.duration_since(detected_at).as_millis();
                                            run.settle_sum_ms += settle_ms;
                                            run.settle_max_ms = run.settle_max_ms.max(settle_ms);
                                            run.settle_count += 1;
                                        }
                                    }

                                    if run.step >= run.cycles.saturating_mul(2) {
                                        let background_mean = run.background_sum_mg as f64
                                            / run.background_samples as f64;
                                        let background_variance = (run.background_sum_sq_mg as f64
                                            / run.background_samples as f64
                                            - background_mean * background_mean)
                                            .max(0.0);
                                        let loaded_mean =
                                            run.loaded_sum_mg as f64 / run.loaded_samples as f64;
                                        let loaded_variance = (run.loaded_sum_sq_mg as f64
                                            / run.loaded_samples as f64
                                            - loaded_mean * loaded_mean)
                                            .max(0.0);
                                        let average_settle_ms =
                                            run.settle_sum_ms / run.settle_count.max(1) as u64;
                                        let background_noise_g =
                                            libm::sqrt(background_variance) / 1000.0;
                                        let loaded_noise_g = libm::sqrt(loaded_variance) / 1000.0;
                                        let average_settle_ms =
                                            average_settle_ms.min(u32::MAX as u64) as u32;
                                        let maximum_settle_ms =
                                            run.settle_max_ms.min(u32::MAX as u64) as u32;
                                        let before = persistent.values;
                                        persistent.values.characterisation_cycles = run.cycles;
                                        persistent.values.characterisation_reference_mass_g =
                                            run.known_mass_g;
                                        persistent.values.characterisation_background_noise_sd_g =
                                            background_noise_g as f32;
                                        persistent.values.characterisation_loaded_mean_g =
                                            (loaded_mean / 1000.0) as f32;
                                        persistent.values.characterisation_loaded_noise_sd_g =
                                            loaded_noise_g as f32;
                                        persistent.values.characterisation_average_settle_ms =
                                            average_settle_ms;
                                        persistent.values.characterisation_maximum_settle_ms =
                                            maximum_settle_ms;
                                        persistent.values.characterisation_valid = true;
                                        let saved = persistent.save(&mut flash).is_ok();
                                        if !saved {
                                            persistent.values = before;
                                        }
                                        log::info!(
                                            "Characterisation {}: {} load cycles, background SD={:.4} g, loaded mean={:.3} g, loaded SD={:.4} g, average settling={} ms, maximum settling={} ms.",
                                            if saved {
                                                "saved"
                                            } else {
                                                "completed but not saved"
                                            },
                                            run.cycles,
                                            background_noise_g,
                                            loaded_mean / 1000.0,
                                            loaded_noise_g,
                                            average_settle_ms,
                                            maximum_settle_ms
                                        );
                                        if run.id != 0 && saved {
                                            log::info!(
                                                "@SCALE/2,EVENT,CHAR_DONE,{},{},{:.4},{:.3},{:.4},{},{}",
                                                run.id,
                                                run.cycles,
                                                background_noise_g,
                                                loaded_mean / 1000.0,
                                                loaded_noise_g,
                                                average_settle_ms,
                                                maximum_settle_ms
                                            );
                                        } else if run.id != 0 {
                                            log::info!("@SCALE/2,ERROR,{},FLASH_ERROR", run.id);
                                        }
                                        keep_run = false;
                                    } else {
                                        run.step += 1;
                                        run.target_loaded = run.step % 2 == 1;
                                        run.prompted_at = sample_time;
                                        run.detected_at = None;
                                        run.reset_window();
                                        let action = if run.target_loaded {
                                            "ADD_LOAD"
                                        } else {
                                            "REMOVE_LOAD"
                                        };
                                        log::info!(
                                            "Characterisation step {}: {}.",
                                            run.step,
                                            action
                                        );
                                        if run.id != 0 {
                                            log::info!(
                                                "@SCALE/2,EVENT,CHAR_STEP,{},{},{}",
                                                run.id,
                                                run.step,
                                                action
                                            );
                                        }
                                    }
                                } else {
                                    run.window_count = 0;
                                    run.window_sum_mg = 0;
                                    run.window_sum_sq_mg = 0;
                                }
                            }
                        }
                    }
                    if keep_run {
                        characterisation = Some(run);
                    }
                }

                DISPLAY_WATCH.sender().send(DisplayData {
                    value: output_calibrated,
                    tare_flag: tare_completed,
                    mode,
                });

                let now = Instant::now();
                let stability_name = match level {
                    StabilityLevel::Unstable => "UNSTABLE",
                    StabilityLevel::Settling => "SETTLING",
                    StabilityLevel::Stable => "STABLE",
                };
                let mode_name = match mode {
                    ScaleMode::Fast => "FAST",
                    ScaleMode::Settling => "SETTLING",
                    ScaleMode::Stable => "STABLE",
                };
                if now - last_ui_update_time >= Duration::from_millis(50) {
                    reading_sequence = reading_sequence.wrapping_add(1);
                    log::info!(
                        "@SCALE/2,READING,{},{:.3},{},{},{}",
                        reading_sequence,
                        output_calibrated,
                        stability_name,
                        mode_name,
                        if tare_completed { 1 } else { 0 }
                    );
                    last_ui_update_time = now;
                }
                if now - last_report_time >= Duration::from_millis(500) {
                    log::info!(
                        "Mode: {:?}, Stability: {:?}, Value: {:.3} g, Variance: {:?} counts^2",
                        mode,
                        level,
                        output_calibrated,
                        stability.measured_variance(),
                    );
                    last_report_time = now;
                }
            }
        }
    }
}

fn send_properties(id: u32, p: &DeviceProperties) {
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.9},g_per_count,RW",
        id,
        "calibration.scale_factor_g_per_count",
        p.scale_factor_g_per_count
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.3},g,RO",
        id,
        "calibration.last_reference_mass_g",
        p.calibration_last_reference_mass_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.1},counts,RO",
        id,
        "calibration.last_raw_delta_counts",
        p.calibration_last_raw_delta_counts
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},counts,RW",
        id,
        "tare.offset_counts",
        p.tare_offset_counts
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},counts2,RW",
        id,
        "tare.noise_variance_counts2",
        p.tare_noise_variance_counts2
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},bool,RO",
        id,
        "tare.valid",
        if p.tare_valid { 1 } else { 0 }
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.3},g,RW",
        id,
        "characterisation.reference_mass_g",
        p.characterisation_reference_mass_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},cycles,RW",
        id,
        "characterisation.cycles",
        p.characterisation_cycles
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.3},g,RW",
        id,
        "characterisation.convergence_g",
        p.characterisation_convergence_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},samples,RW",
        id,
        "characterisation.window_samples",
        p.characterisation_window_samples
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},bool,RO",
        id,
        "characterisation.valid",
        if p.characterisation_valid { 1 } else { 0 }
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.4},g,RO",
        id,
        "characterisation.background_noise_sd_g",
        p.characterisation_background_noise_sd_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.3},g,RO",
        id,
        "characterisation.loaded_mean_g",
        p.characterisation_loaded_mean_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{:.4},g,RO",
        id,
        "characterisation.loaded_noise_sd_g",
        p.characterisation_loaded_noise_sd_g
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},ms,RO",
        id,
        "characterisation.average_settle_ms",
        p.characterisation_average_settle_ms
    );
    log::info!(
        "@SCALE/2,PROPERTY,{},{},{},ms,RO",
        id,
        "characterisation.maximum_settle_ms",
        p.characterisation_maximum_settle_ms
    );
    log::info!("@SCALE/2,PROPERTY_END,{},16", id);
}
