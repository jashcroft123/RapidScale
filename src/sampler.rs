use embassy_rp::gpio::Input;
use embassy_rp::i2c;
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use variegated_nau7802::{Gain, Ldo, Nau7802, Nau7802DataAvailableStrategy, SamplesPerSecond};

pub type ADCChannel = Channel<CriticalSectionRawMutex, i32, 128>;
pub static CHANNEL: ADCChannel = Channel::new();

#[embassy_executor::task]
pub async fn sampler_task(i2c: i2c::I2c<'static, I2C0, i2c::Async>, drdy_pin: Input<'static>) {
    let strategy = Nau7802DataAvailableStrategy::DrdyPin(drdy_pin);
    let mut nau = Nau7802::new(i2c, strategy, embassy_time::Delay, Some(0x2A));

    if let Err(e) = nau
        .init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS40)
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
