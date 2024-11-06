pub const V5_BRAIN_USB_PID: u16 = 0x0501;
pub const V5_USB_VID: u16 = 0x2888;

pub const MAX_PKT_SIZE: usize = 2047;

pub static LAST_UPDATE: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);

pub use protocol;

pub struct BrainMediator {
    port: Box<dyn serialport::SerialPort>,
    read_buffer: Vec<u8>,
    partial_read_buffer: [u8; MAX_PKT_SIZE],
}

impl BrainMediator {
    pub fn new() -> Option<Self> {
        let port = serialport::available_ports()
            .unwrap()
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
            })?;

        let port = serialport::new(port.port_name, 115200)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::None)
            .data_bits(serialport::DataBits::Eight)
            .open()
            .unwrap();

        Some(Self {
            port,
            read_buffer: Vec::new(),
            partial_read_buffer: [0; MAX_PKT_SIZE],
        })
    }
    pub fn try_read(&mut self) -> Result<Vec<protocol::ToRobot>, Box<dyn std::error::Error>> {
        let mut pkts = Vec::new();
        while self.port.bytes_to_read()? != 0 {
            if let Ok(read) = self.port.read(&mut self.partial_read_buffer) {
                if read > MAX_PKT_SIZE {
                    log::warn!("read too big, dropping!");
                    continue;
                } else if read + self.read_buffer.len() > MAX_PKT_SIZE {
                    self.read_buffer.clear();
                }

                self.read_buffer.extend(&self.partial_read_buffer[..read]);
                if let Some(idx) = self.read_buffer.iter().position(|&v| v == 0) {
                    let tbuf = self.read_buffer.drain(..=idx).collect::<Vec<_>>();
                    let Ok(v) = postcard::from_bytes_cobs::<protocol::ToRobot>(&mut tbuf.clone())
                    else {
                        log::warn!("Parse error with: {tbuf:?}");
                        continue;
                    };

                    pkts.push(v);
                }
            }
        }
        Ok(pkts)
    }
    pub fn try_write(&mut self, v: &protocol::ToBrain) -> Result<(), Box<dyn std::error::Error>> {
        let option = &mut *LAST_UPDATE.try_lock()?;

        if let Some(time) = option.as_mut() {
            if time.elapsed() < std::time::Duration::from_millis(2) {
                return Ok(());
            }
        }

        let data = postcard::to_stdvec_cobs(&v)?;
        if data.len() > MAX_PKT_SIZE {
            unreachable!();
        }

        *option = Some(std::time::Instant::now());
        Ok(self.port.write_all(&data)?)
    }
}
