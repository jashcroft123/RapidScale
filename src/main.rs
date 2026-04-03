#![no_std]
#![no_main]

mod Filtering;

use Filtering::{EMA, Filter, FilterStack, Gaussian, Median, SMA};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;
use variegated_nau7802::{Gain, Ldo, Nau7802, Nau7802DataAvailableStrategy, SamplesPerSecond};

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{Dimensions, Size};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use mipidsi::{Builder, interface::SpiInterface};

// ------------------------
// Interrupt bindings
// ------------------------
bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

bind_interrupts!(struct I2cIrqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// ------------------------
// USB logger task
// ------------------------
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

// ------------------------
// ADC channel
// ------------------------
type ADCChannel = Channel<CriticalSectionRawMutex, i32, 128>;
static CHANNEL: ADCChannel = Channel::new();

// ------------------------
// Event-driven sampler task
// ------------------------
#[embassy_executor::task]
async fn sampler_task(i2c: i2c::I2c<'static, I2C0, i2c::Async>, drdy_pin: Input<'static>) {
    let strategy = Nau7802DataAvailableStrategy::DrdyPin(drdy_pin);
    let mut nau = Nau7802::new(i2c, strategy, embassy_time::Delay, Some(0x2A));

    if let Err(e) = nau
        .init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS80)
        .await
    {
        log::error!("Sampler: Failed to initialize NAU7802: {:?}", e);
        return;
    }

    log::info!("Sampler: NAU7802 initialized at 320 SPS. Starting loop.");

    loop {
        if let Ok(()) = nau.wait_for_data_available().await {
            if let Ok(reading) = nau.read().await {
                if let Err(_) = CHANNEL.try_send(reading) {
                    // This is fine, just means main is busy
                }
            } else {
                log::error!("Sampler: Failed to read from NAU7802");
            }
        } else {
            log::warn!("Sampler: Wait for data timed out or failed");
        }
    }
}

// ------------------------
// Scale mode enum
// ------------------------
#[derive(Debug, Clone, PartialEq, Eq)]
enum ScaleMode {
    Fast,
    Stable,
}

enum ScaleState {
    Tare,
    Reading,
}

static DISPLAY_WIDTH: u32 = 172;
static DISPLAY_HEIGHT: u32 = 320;
static mut CORE1_STACK: Stack<8192> = Stack::new();
static mut CORE1_EXECUTOR: core::mem::MaybeUninit<embassy_executor::Executor> =
    core::mem::MaybeUninit::uninit();
static mut DISPLAY_BUFFER: [u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2) as usize] =
    [0_u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2) as usize];

#[derive(Clone, Copy)]
struct DisplayData {
    value: f32,
    tare_flag: bool,
}

static DISPLAY_WATCH: Watch<CriticalSectionRawMutex, DisplayData, 1> = Watch::new();
static mut TEXT_BUF_DATA: [Rgb565; 280 * 40] = [Rgb565::BLACK; 280 * 40];

struct TextBuf<'a> {
    data: &'a mut [Rgb565],
    width: i32,
    height: i32,
}
impl<'a> Dimensions for TextBuf<'a> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(
            Point::zero(),
            Size::new(self.width as u32, self.height as u32),
        )
    }
}
impl<'a> DrawTarget for TextBuf<'a> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(Point { x, y }, color) in pixels {
            if x >= 0 && x < self.width && y >= 0 && y < self.height {
                self.data[(y * self.width + x) as usize] = color;
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let intersection = self.bounding_box().intersection(area);
        for y in intersection.rows() {
            for x in intersection.columns() {
                self.data[(y * self.width + x) as usize] = color;
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
async fn display_task(
    spi: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    dc: Output<'static>,
    cs: Output<'static>,
    rst: Output<'static>,
) {
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs);
    let di = SpiInterface::new(spi_device, dc, unsafe {
        &mut *core::ptr::addr_of_mut!(DISPLAY_BUFFER)
    });

    let mut display = match Builder::new(mipidsi::models::ST7789, di)
        .display_size(DISPLAY_WIDTH as u16, DISPLAY_HEIGHT as u16)
        .display_offset(0, 0)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .reset_pin(rst)
        .init(&mut embassy_time::Delay)
    {
        Ok(d) => d,
        Err(e) => {
            log::error!("Display: Failed to initialize ST7789: {:?}", e);
            return;
        }
    };

    if let Err(e) = display.clear(Rgb565::BLACK) {
        log::error!("Display: Failed to clear display: {:?}", e);
    }

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::CSS_ORANGE)
        .background_color(Rgb565::BLACK)
        .build();

    let value_style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_24_POINT)
        .text_color(Rgb565::CSS_ORANGE)
        .background_color(Rgb565::BLACK)
        .build();

    if let Err(e) = Text::new("Scale booting...", Point::new(10, 20), text_style).draw(&mut display)
    {
        log::error!("Display: Failed to draw initial text: {:?}", e);
    }

    let mut last_tare = false;
    let mut frame_count = 0;
    let mut last_fps_time = Instant::now();
    let mut max_fps = 0;

    let mut rx = DISPLAY_WATCH
        .receiver()
        .expect("Display: Watch receiver was None");
    let text_buf_data = unsafe { &mut *core::ptr::addr_of_mut!(TEXT_BUF_DATA) };

    loop {
        let data = rx.changed().await;

        let now = Instant::now();
        frame_count += 1;
        let update_fps = now - last_fps_time >= Duration::from_secs(1);
        if update_fps {
            max_fps = frame_count;
            frame_count = 0;
            last_fps_time = now;
        }

        if data.tare_flag && !last_tare {
            let _ = display.clear(Rgb565::BLACK);
            last_tare = true;
        }

        use core::fmt::Write;

        let mut text_buf = TextBuf {
            data: &mut *text_buf_data,
            width: 280,
            height: 40,
        };

        let mut buf: heapless::String<64> = heapless::String::new();
        let _ = write!(&mut buf, "{:8.3} g", data.value);
        let _ = Text::new(&buf, Point::new(0, 20), value_style).draw(&mut text_buf);
        let _ = display.fill_contiguous(
            &Rectangle::new(Point::new(10, 50), Size::new(280, 40)),
            text_buf.data.iter().copied(),
        );

        // if update_fps {
        //     let mut fps_tb = TextBuf {
        //         data: &mut text_buf.data[0..100 * 30],
        //         width: 100,
        //         height: 30,
        //     };
        //     let _ = fps_tb.fill_solid(&fps_tb.bounding_box(), Rgb565::BLACK);
        //     let mut fps_buf: heapless::String<32> = heapless::String::new();
        //     let _ = write!(&mut fps_buf, "FPS: {}", max_fps);
        //     let _ = Text::new(&fps_buf, Point::new(0, 20), text_style).draw(&mut fps_tb);
        //     let _ = display.fill_contiguous(
        //         &Rectangle::new(Point::new(210, 65), Size::new(100, 30)),
        //         fps_tb.data.iter().copied(),
        //     );
        // }

        // if now - last_graph_time >= Duration::from_millis(333) {
        //     for i in 0..299 {
        //         history[i] = history[i + 1];
        //     }
        //     history[299] = data.value;

        //     let mut min_val = history[0];
        //     let mut max_val = history[0];
        //     for &val in history.iter() {
        //         if val < min_val {
        //             min_val = val;
        //         }
        //         if val > max_val {
        //             max_val = val;
        //         }
        //     }
        //     if max_val - min_val < 0.1 {
        //         max_val += 0.05;
        //         min_val -= 0.05;
        //     }
        //     let range = max_val - min_val;

        //     let mut graph_buf = TextBuf {
        //         data: &mut *graph_buf_data,
        //         width: 300,
        //         height: 60,
        //     };
        //     let _ = graph_buf.fill_solid(&graph_buf.bounding_box(), Rgb565::BLACK);

        //     for i in 1..300 {
        //         let y1 = libm::roundf(59.0 - ((history[i - 1] - min_val) / range) * 59.0) as i32;
        //         let y2 = libm::roundf(59.0 - ((history[i] - min_val) / range) * 59.0) as i32;

        //         let _ = Line::new(Point::new((i - 1) as i32, y1), Point::new(i as i32, y2))
        //             .into_styled(PrimitiveStyle::with_stroke(Rgb565::CYAN, 1))
        //             .draw(&mut graph_buf);
        //     }

        //     let _ = Rectangle::new(Point::zero(), Size::new(300, 60))
        //         .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        //         .draw(&mut graph_buf);

        //     let _ = display.fill_contiguous(
        //         &Rectangle::new(Point::new(10, 105), Size::new(300, 60)),
        //         graph_buf.data.iter().copied(),
        //     );

        //     last_graph_time = now;
        // }
    }
}

// ------------------------
// Main entry
// ------------------------
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // USB logger
    let driver = Driver::new(p.USB, UsbIrqs);
    if let Err(e) = spawner.spawn(logger_task(driver)) {
        // If we can't spawn the logger, we can't really log. Just panic as it's a fatal setup error.
        panic!("Failed to spawn logger task: {:?}", e);
    }
    Timer::after(Duration::from_millis(2000)).await;
    log::info!("Starting scale...");

    // I2C configuration
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000; // Standard-Plus speed (800k is often too fast for NAU)
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, I2cIrqs, i2c_config);

    // Spawn sampler task
    let drdy_pin = Input::new(p.PIN_6, Pull::Up);
    if let Err(e) = spawner.spawn(sampler_task(i2c, drdy_pin)) {
        log::error!("Main: Failed to spawn sampler task: {:?}", e);
    }

    // ------------------------
    // LCD Init transferred to core 1 task
    // ------------------------
    let mut display_spi_config = SpiConfig::default();
    display_spi_config.frequency = 62_500_000;

    // SPI0: SCK=18, MOSI=19
    let display_spi = Spi::new_blocking_txonly(p.SPI0, p.PIN_18, p.PIN_19, display_spi_config);

    let dc = Output::new(p.PIN_16, Level::Low);
    let cs = Output::new(p.PIN_17, Level::High);
    let rst = Output::new(p.PIN_20, Level::Low);
    let mut _bl = Output::new(p.PIN_21, Level::High); // Turn on backlight

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

    // let mut average_value: SMA<2048> = SMA::new();

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

    loop {
        // Log every few seconds if we are stuck waiting for data from the sampler
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
                // Detect change
                let change = last_value.map_or(0, |v| (reading - v).abs());
                last_value = Some(reading);

                // State transition
                let new_mode = if change > 10000 {
                    ScaleMode::Fast
                } else {
                    ScaleMode::Stable
                };

                if mode == ScaleMode::Stable && new_mode == ScaleMode::Fast {
                    average_value.reset();
                }
                mode = new_mode;

                // SMA calculation or passthrough
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

                // Report every 500 ms to log
                let now = Instant::now();
                if now - last_report_time >= Duration::from_millis(500) {
                    log::info!(
                        "Mode: {:?}, Value: {:.3}, Change: {}, saturated: {}",
                        mode,
                        output_calibrated,
                        change,
                        saturated,
                    );
                    last_report_time = now;
                }
            }
        }
    }
}
