use core::fmt::Write;
use embassy_rp::gpio::Output;
use embassy_rp::multicore::Stack;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{Dimensions, Size};
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use mipidsi::Builder;
use mipidsi::interface::SpiInterface;

use crate::types::DisplayData;

pub const DISPLAY_WIDTH: u32 = 172;
pub const DISPLAY_HEIGHT: u32 = 320;

pub static mut CORE1_STACK: Stack<8192> = Stack::new();
pub static mut CORE1_EXECUTOR: core::mem::MaybeUninit<embassy_executor::Executor> =
    core::mem::MaybeUninit::uninit();
pub static mut DISPLAY_BUFFER: [u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2) as usize] =
    [0_u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2) as usize];

pub static DISPLAY_WATCH: Watch<CriticalSectionRawMutex, DisplayData, 1> = Watch::new();
pub static mut TEXT_BUF_DATA: [Rgb565; 280 * 40] = [Rgb565::BLACK; 280 * 40];

pub struct TextBuf<'a> {
    pub data: &'a mut [Rgb565],
    pub width: i32,
    pub height: i32,
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
pub async fn display_task(
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
    let mut _max_fps = 0;

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
            _max_fps = frame_count;
            frame_count = 0;
            last_fps_time = now;
        }

        if data.tare_flag && !last_tare {
            let _ = display.clear(Rgb565::BLACK);
            last_tare = true;
        }

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
    }
}
