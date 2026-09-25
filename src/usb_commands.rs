use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;

static USB_BYTES: Channel<CriticalSectionRawMutex, u8, 128> = Channel::new();
pub static USB_COMMANDS: Channel<CriticalSectionRawMutex, String<128>, 4> = Channel::new();

pub struct CommandReceiver;

impl embassy_usb_logger::ReceiverHandler for CommandReceiver {
    async fn handle_data(&self, data: &[u8]) {
        for byte in data {
            USB_BYTES.send(*byte).await;
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
pub async fn command_parser_task() {
    let mut line: String<128> = String::new();
    loop {
        let byte = USB_BYTES.receive().await;
        match byte {
            b'\r' => {}
            b'\n' => {
                if !line.is_empty() {
                    USB_COMMANDS.send(line).await;
                    line = String::new();
                }
            }
            8 | 127 => {
                line.pop();
            }
            32..=126 => {
                if line.push(byte as char).is_err() {
                    line.clear();
                    log::warn!("USB command too long; discarded. Maximum is 127 characters.");
                }
            }
            _ => {}
        }
    }
}
