pub const V5_BRAIN_USB_PID: u16 = 0x0501;
pub const V5_USB_VID: u16 = 0x2888;

pub const MAX_PKT_SIZE: usize = 2047;

use std::sync::{Mutex, MutexGuard, TryLockError};

pub static LAST_UPDATE: Mutex<Option<std::time::Instant>> = Mutex::new(None);

pub use protocol;

pub struct BrainMediator {
    port: Box<dyn serialport::SerialPort>,
    read_buffer: Vec<u8>,
    partial_read_buffer: [u8; MAX_PKT_SIZE],
}

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("COBS error: {0}")]
    Cobs(#[from] postcard::Error),
    #[error("No packets read")]
    NoPacketRead,
    #[error("Failed to lock mutex for LAST_UPDATE")]
    MutexLock(#[from] TryLockError<MutexGuard<'static, Option<std::time::Instant>>>),
    #[error("Serial Writing Error: {0}")]
    SerialWriteError(#[from] std::io::Error),
    #[error("Brain not found")]
    BrainNotFound,
    #[error("Serial Error: {0}")]
    SerialError(#[from] serialport::Error),
    #[error("Unreachable code reached {0}")]
    Unreachable(&'static str),
}

impl BrainMediator {
    pub fn new() -> Result<Self, Error> {
        let port = serialport::available_ports()?
            .into_iter()
            .find(|p| {
                if let serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
                    vid,
                    pid,
                    interface,
                    ..
                }) = &p.port_type
                {
                    // look for user program connection (interface == 2)
                    *pid == V5_BRAIN_USB_PID && *vid == V5_USB_VID && interface == &Some(2)
                } else {
                    false
                }
            })
            .ok_or(Error::BrainNotFound)?;

        let port = serialport::new(port.port_name, 115_200)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::None)
            .data_bits(serialport::DataBits::Eight)
            .open()?;

        Ok(Self {
            port,
            read_buffer: Vec::new(),
            partial_read_buffer: [0; MAX_PKT_SIZE],
        })
    }
    // only read single packet (assume reader can write faster then brain can write)
    pub fn try_read(&mut self) -> Result<protocol::ToRobot, Error> {
        while self.port.bytes_to_read()? != 0 {
            // update read buffer
            if let Ok(read) = self.port.read(&mut self.partial_read_buffer) {
                if read > MAX_PKT_SIZE {
                    log::warn!("Read too big, dropping!");
                    continue;
                } else if read + self.read_buffer.len() > MAX_PKT_SIZE {
                    log::warn!(
                        "Total read buffer size exceeds MAX_PKT_SIZE ({MAX_PKT_SIZE}. Clearing"
                    );
                    self.read_buffer.clear();
                }
                self.read_buffer.extend(&self.partial_read_buffer[..read]);
            }

            // find cobs delimiter and parse that section of the read buffer into a packet
            if let Some(idx) = self.read_buffer.iter().position(|&e| e == 0) {
                let temp_buf: Vec<_> = self.read_buffer.drain(..=idx).collect();
                let Ok(pkt) = postcard::from_bytes_cobs::<protocol::ToRobot>(&mut temp_buf.clone())
                else {
                    log::warn!("Parse error with: {temp_buf:?}");
                    continue;
                };
                return Ok(pkt);
            }
        }
        Err(Error::NoPacketRead)
    }
    pub fn try_write(&mut self, v: &protocol::ToBrain) -> Result<(), Error> {
        let option = &mut *LAST_UPDATE.try_lock()?;

        if let Some(time) = option.as_mut() {
            if time.elapsed() < std::time::Duration::from_millis(2) {
                return Ok(());
            }
        }

        let data = postcard::to_stdvec_cobs(&v)?;
        if data.len() > MAX_PKT_SIZE {
            return Err(Error::Unreachable("try_write: data.len() > MAX_PKT_SIZE"));
        }

        *option = Some(std::time::Instant::now());
        Ok(self.port.write_all(&data)?)
    }
}
