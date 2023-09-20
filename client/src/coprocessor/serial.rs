use std::{
	fmt::Write as FmtWrite,
	io::{self, BufRead, BufReader, BufWriter, Read, Write},
	path::Path,
	sync::{Arc, LockResult, Mutex, MutexGuard},
	time::{Duration, Instant},
};

use protocol::{
	self as proto, ControlPkt, Devices, ErrorPkt, ErrorType, GenericPkt, InitPkt, Packet, StatusPkt,
};
use serialport::{SerialPortInfo, SerialPortType, TTYPort, UsbPortInfo};

pub fn find_v5_port() -> serialport::Result<(SerialPortInfo, SerialPortInfo)> {
	let mut user_port = None;
	let mut system_port = None;

	for port in serialport::available_ports()?
		.into_iter()
		.filter(|p| match &p.port_type {
			SerialPortType::UsbPort(UsbPortInfo {
				vid,
				pid: _,
				serial_number: _,
				manufacturer: _,
				product,
			}) => {
				[0x2888, 0x0501].contains(vid)
					| product
						.as_ref()
						.map(|s| s.contains("VEX") | s.contains("V5"))
						.is_some()
			}
			// usb ports are consider PCI on the rpi
			SerialPortType::PciPort => true,
			_ => false,
		}) {
		let interface = std::fs::read(format!(
			"/sys/class/tty/{}/device/interface",
			Path::new(&port.port_name)
				.file_name()
				.unwrap()
				.to_string_lossy()
		))
		.ok()
		.map(|v| String::from_utf8(v).unwrap());

		if let Some(interface) = interface {
			if interface.contains("Communications Port") {
				system_port = Some(port);
			} else if interface.contains("User Port") {
				user_port = Some(port);
			}
		}
	}

	match (user_port, system_port) {
		(Some(user), Some(system)) => Ok((user, system)),
		_ => Err(serialport::Error::new(
			serialport::ErrorKind::Unknown,
			String::from("Could not find two valid ports"),
		)),
	}
}

pub struct Serial {
	reader: BufReader<TTYPort>,
	writer: BufWriter<TTYPort>,
}

impl Serial {
	pub fn open(path: &str) -> serialport::Result<Self> {
		let port = serialport::new(path, 115200)
			.parity(serialport::Parity::None)
			.stop_bits(serialport::StopBits::One)
			.data_bits(serialport::DataBits::Eight)
			.timeout(Duration::from_secs(1))
			.open_native()
			.and_then(|mut tty| {
				tty.set_exclusive(false)?;
				Ok(tty)
			})?;

		let reader = BufReader::new(port.try_clone_native()?);
		let writer = BufWriter::new(port);

		Ok(Self { reader, writer })
	}

	pub fn set_exclusive(&mut self, exclusive: bool) -> serialport::Result<()> {
		self.reader.get_mut().set_exclusive(exclusive)?;
		self.writer.get_mut().set_exclusive(exclusive)?;

		Ok(())
	}
}

impl Serial {
	pub fn spawn_threaded(self) -> SerialData {
		let data = SerialData::new(&Devices::default());

		let x = data.clone();
		std::thread::spawn(move || Self::serial_handler(x, self));

		data
	}

	pub fn serial_handler(data: SerialData, this: Self) {
		let mut reader = this.reader;
		let mut writer = this.writer;

		let mut read_buf = vec![0; 512];
		let mut write_buf = vec![0; 512].into_boxed_slice();
		let mut cobs_buf = vec![0; 512].into_boxed_slice();

		let mut state = State::BeginInit;
		let mut timeout_count = 0;
		const TIMEOUT: usize = 3;

		'outer: loop {
			match state {
				// Send an InitPkt
				State::BeginInit => {
					send_pkt(
						&mut writer,
						&mut write_buf,
						&mut cobs_buf,
						InitPkt::default(),
					)
					.unwrap();
					state = State::WaitingForDevices;
					continue;
				}
				State::WaitingForDevices => {
					// Read until we get a valid devices packet
					let devices;
					let mut timeout_count = 0;
					loop {
						match recv_pkt(
							&mut reader,
							&mut read_buf,
							&mut cobs_buf,
							&Devices::default(),
						) {
							Ok(pkt) => {
								devices = pkt;
								log::debug!("Read devices packet: {:?}", devices);
								break;
							}
							Err(Error::IoErr(err))
								if matches!(err.kind(), std::io::ErrorKind::TimedOut) =>
							{
								timeout_count += 1;
								// If a respone is not sent in 3 second than try and send another
								// init packet
								if timeout_count >= TIMEOUT {
									log::info!("InitPkt timeout, retrying");
									state = State::BeginInit;
									continue 'outer;
								}
							}
							Err(err) => {
								log::debug!("Failed to read devices packet: {:?}", err);
							}
						}
					}
					// Move to next state
					state = State::Operating(devices);
					continue;
				}
				State::Operating(ref devices) => {
					// Write outgoing control packet
					let pkt = { data.send_pkt_lock().unwrap().clone() };
					match send_pkt(&mut writer, &mut write_buf, &mut cobs_buf, pkt) {
						Err(err) => {
							log::error!("Failed to send packet: {:?}", err);
							return;
						}
						_ => {}
					}

					// Read incoming packet
					match recv_pkt::<GenericPkt>(
						&mut reader,
						&mut read_buf,
						&mut cobs_buf,
						&devices,
					) {
						Ok(GenericPkt::StatusPkt(pkt)) => {
							*data.recv_pkt_lock().unwrap() = pkt;
						}
						Ok(GenericPkt::ErrorPkt(ErrorPkt {
							err: ErrorType::Recoverable,
						})) => {
							log::error!("Recoverable error packet sent from secondary controller, waiting for device list");
							state = State::WaitingForDevices;
							continue;
						}
						Ok(GenericPkt::ErrorPkt(ErrorPkt {
							err: ErrorType::Fatal,
						})) => {
							log::error!("Fatal error packet sent from secondary controller");
							state = State::FatalError(Error::SecondaryError);
							continue;
						}
						Ok(_) => {
							// Other packet types are unexpected here
							unimplemented!()
						}
						Err(Error::IoErr(err))
							if matches!(err.kind(), std::io::ErrorKind::TimedOut) =>
						{
							timeout_count += 1;
							// If a respone is not sent in 3 second than try and send another
							// init packet
							if timeout_count >= TIMEOUT {
								log::info!("StatusPkt timeout, retrying");
								state = State::BeginInit;
								timeout_count = 0;
								continue 'outer;
							}
						}
						Err(err) => {
							log::error!("Failed to read packet: {:?}", err);
							// TODO: perhaps try restarting the connection?
							state = State::BeginInit;
							continue 'outer;
						}
					}
				}
				State::FatalError(err) => {
					log::error!("Fatal error occurred: {:?}", err);
					return;
				}
			}

			// Wait a bit until we write another packet
			std::thread::sleep(Duration::from_millis(5));
		}
	}
}

#[derive(Debug)]
enum State {
	BeginInit,
	WaitingForDevices,
	Operating(Devices),
	FatalError(Error),
}

/// This encodes and sends a COBS message to the V5 brain. It will clobber `cobs_buf`.
fn send_msg(
	writer: &mut BufWriter<TTYPort>,
	message: &[u8],
	cobs_buf: &mut Box<[u8]>,
) -> Result<(), Error> {
	// Make sure there is enough space
	let needed_len = cobs::max_encode_size(message.len());
	if cobs_buf.len() < needed_len {
		*cobs_buf = vec![0; needed_len].into_boxed_slice();
	}
	// Encode into COBS
	let bytes = cobs::encode(message, cobs_buf)?;

	// Write with NUL terminator
	writer.write_all(bytes)?;
	writer.write_all(&[0])?;
	writer.flush()?;

	Ok(())
}

fn send_pkt<P: Packet>(
	writer: &mut BufWriter<TTYPort>,
	write_buf: &mut Box<[u8]>,
	cobs_buf: &mut Box<[u8]>,
	pkt: P,
) -> Result<(), Error> {
	let bytes = pkt.serialise(write_buf)?;
	send_msg(writer, bytes, cobs_buf)?;

	Ok(())
}

/// This will receive and decode a COBS message from the V5 Brain. It will clobber both `read_buf` and `cobs_buf`
fn recv_msg<'a>(
	reader: &mut BufReader<TTYPort>,
	read_buf: &mut Vec<u8>,
	cobs_buf: &'a mut Box<[u8]>,
) -> Result<&'a [u8], Error> {
	// Read bytes until 0
	read_buf.clear();
	let bytes_len = reader.read_until(0, read_buf)?;
	let bytes = &read_buf[..bytes_len - 1];

	// Decode from COBS
	let needed_len = cobs::max_decode_size(bytes.len());
	if cobs_buf.len() < needed_len {
		*cobs_buf = vec![0; needed_len].into_boxed_slice();
	}
	let message = cobs::decode(bytes, cobs_buf)?;

	Ok(message)
}

fn recv_pkt<P: Packet + std::fmt::Debug>(
	reader: &mut BufReader<TTYPort>,
	read_buf: &mut Vec<u8>,
	cobs_buf: &mut Box<[u8]>,
	devices: &Devices,
) -> Result<P, Error> {
	let msg;
	loop {
		// Read message
		let bytes = recv_msg(reader, read_buf, cobs_buf)?;
		// Read until we get a valid looking message
		if bytes != b"sout\n" {
			msg = bytes;
			break;
		}
	}

	match P::deserialise(devices, msg) {
		Ok(pkt) => Ok(pkt),
		Err(err) => {
			log::debug!("Failed to decode packet ({:?})\nbytes: {:?}", err, msg);
			Err(err.into())
		}
	}
}

#[derive(Clone)]
pub struct SerialData(Arc<SerialDataInner>);

struct SerialDataInner {
	recv: Mutex<StatusPkt>,
	send: Mutex<ControlPkt>,
}

impl SerialData {
	pub fn new(devices: &Devices) -> Self {
		Self(Arc::new(SerialDataInner {
			recv: Mutex::new(StatusPkt::from_devices(devices)),
			send: Mutex::new(ControlPkt::from_devices(devices)),
		}))
	}

	pub fn recv_pkt_lock(&self) -> LockResult<MutexGuard<'_, StatusPkt>> {
		self.0.recv.lock()
	}

	pub fn send_pkt_lock(&self) -> LockResult<MutexGuard<'_, ControlPkt>> {
		self.0.send.lock()
	}
}

#[derive(Debug)]
enum Error {
	CobsErr(cobs::Error),
	IoErr(io::Error),
	PacketErr(proto::Error),
	SecondaryError,
}

impl std::fmt::Display for Error {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
		write!(f, "{:?}", self)
	}
}

impl std::error::Error for Error {}

impl From<proto::Error> for Error {
	fn from(x: proto::Error) -> Self {
		Self::PacketErr(x)
	}
}

impl From<cobs::Error> for Error {
	fn from(x: cobs::Error) -> Self {
		Self::CobsErr(x)
	}
}

impl From<io::Error> for Error {
	fn from(x: io::Error) -> Self {
		Self::IoErr(x)
	}
}

pub fn rtt_test(
	port: &mut Serial,
	round_count: usize,
	msg_size: usize,
	msg_byte: u8,
) -> Result<Vec<f32>, String> {
	// Create and encode our message
	let msg = std::iter::repeat(msg_byte)
		.take(msg_size)
		.collect::<Vec<_>>();
	let mut msg_encoded = cobs::encode_to_vec(&msg).unwrap();
	msg_encoded.push(0);

	let mut read_buf = vec![0u8; msg_encoded.len()];
	let mut decode_buf = vec![0u8; msg.len() + 1]; // zero byte on end

	let mut rounds = Vec::with_capacity(round_count);

	for i in 0..round_count {
		let start = Instant::now();
		// send msg
		if let Err(e) = port.writer.write_all(&msg_encoded) {
			return Err(format!("round: {}, write: ({:?})", i, e));
		}
		// flush it
		if let Err(e) = port.writer.flush() {
			return Err(format!("round: {}, write: ({:?})", i, e));
		}
		// read exact bytes back
		let len = match port.reader.read(&mut read_buf) {
			Ok(len) => len,
			Err(e) => return Err(format!("round: {}, read: ({:?})", i, e)),
		};
		let duration = start.elapsed().as_micros() as f32 / 1_000.0;
		rounds.push(duration);

		// decode the msg
		let decoded = match cobs::decode(&read_buf[..len - 1], &mut decode_buf) {
			Ok(x) => x,
			Err(err) => {
				return Err(format!(
					"round: {}, failed to decode message: {}, bytes: {:?}",
					i, err, read_buf
				))
			}
		};

		// check they are equal
		if msg != decoded {
			return Err(format!("round: {}, loopback not equal", i));
		}
	}

	Ok(rounds)
}

pub fn rtt_score(times: &[f32]) -> String {
	// Obtain basic statistics about the data
	let (min, max, mean) = {
		let mut min = f32::INFINITY;
		let mut max = f32::NEG_INFINITY;
		let mut sum = 0.0;

		for &v in times {
			min = f32::min(min, v);
			max = f32::max(max, v);
			sum += v;
		}
		(min, max, sum / times.len() as f32)
	};

	// Calculate the population variance
	let variance = {
		let sum: f32 = times.iter().map(|v| f32::powi(v - mean, 2)).sum();
		sum / times.len() as f32
	};

	// Get the head and tail values
	let list_fmt = |values: &[f32]| -> String {
		let mut s = String::new();
		write!(&mut s, "{:.3}", values[0]).ok();
		for v in &values[1..] {
			write!(&mut s, ",{:.3}", v).ok();
		}
		s
	};
	let head = list_fmt(&times[..4]);
	let tail = list_fmt(&times[times.len() - 4..]);

	format!(
		"rtt min/max/mean/variance = {:.3}/{:.3}/{:.3}/{:.3} ms\n\
		rtt head/tail = {}/{} ms\n",
		min, max, mean, variance, head, tail
	)
}
