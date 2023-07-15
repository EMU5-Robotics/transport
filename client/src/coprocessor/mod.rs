use std::{
	io::{self, BufRead, BufReader, BufWriter, Read, Write},
	sync::{Arc, LockResult, Mutex, MutexGuard},
	time::{Duration, Instant},
};

use serialport::{SerialPortInfo, SerialPortType, TTYPort, UsbPortInfo};

use protocol::{self as proto, Layout, Packet, Packet1, Packet2, Packet3, Packet4};

pub fn find_v5_ports() -> serialport::Result<Vec<SerialPortInfo>> {
	// Find and somehow prefer the second interface it creates? I believe that
	// out of `/dev/ttyACM0` and `/dev/ttyACM1` the second is preferable and
	// more correct for communication.

	Ok(serialport::available_ports()?
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
			_ => false,
		})
		.collect())
}

pub struct Port {
	reader: BufReader<TTYPort>,
	writer: BufWriter<TTYPort>,
}

impl Port {
	pub fn open(path: &str) -> serialport::Result<Port> {
		let port = serialport::new(path, 115200)
			.parity(serialport::Parity::None)
			.stop_bits(serialport::StopBits::One)
			.data_bits(serialport::DataBits::Eight)
			.timeout(Duration::from_secs(5))
			.open_native()
			.and_then(|mut tty| {
				tty.set_exclusive(false)?;
				Ok(tty)
			})?;

		let reader = BufReader::new(port.try_clone_native()?);
		let writer = BufWriter::new(port);

		Ok(Port { reader, writer })
	}

	pub fn set_exclusive(&mut self, exclusive: bool) -> serialport::Result<()> {
		self.reader.get_mut().set_exclusive(exclusive)?;
		self.writer.get_mut().set_exclusive(exclusive)?;

		Ok(())
	}

	pub fn spawn_threaded(mut self) -> SerialData {
		let layout = self.serial_handshake();

		log::debug!("Layout: {:#?}", layout);

		let serial_data = SerialData::new(&layout);
		let data = serial_data.clone();

		std::thread::spawn(move || {
			let mut read_buf = vec![];
			let mut write_buf = vec![];
			let mut cobs_buf = vec![];

			loop {
				let pkt = self
					.recv_packet::<Packet3>(&layout, &mut read_buf, &mut cobs_buf)
					.expect("Failed to read packet");
				log::debug!("recv packet: {:?}", pkt);
				{
					*serial_data.recv_pkt_lock().unwrap() = pkt;
				}
				let pkt = { serial_data.send_pkt_lock().unwrap().clone() };
				_ = self
					.send_packet(&pkt, &mut write_buf, &mut cobs_buf)
					.expect("Failed to write packet");

				std::thread::sleep(Duration::from_millis(5));
			}
		});

		data
	}

	fn serial_handshake(&mut self) -> Layout {
		let mut read_buf = vec![];
		let mut write_buf = vec![];
		let mut cobs_buf = vec![];

		// Read until we get a valid packet
		let mut layout = None;
		for _attempt in 0..20 {
			match self.recv_packet::<Packet1>(&Layout::default(), &mut read_buf, &mut cobs_buf) {
				Ok(pkt) => {
					layout = Some(pkt);
					break;
				}
				Err(err) => {
					log::debug!("Failed to read handshake packet: {:?}", err)
				}
			}
		}

		// If we fail to get a valid packet within a set amount of attempts then exit
		let layout = match layout {
			Some(x) => x,
			None => {
				log::error!("Failed to read handshake packet from V5 Brain over USB, exiting");
				std::process::exit(1);
			}
		};

		// Send back an ACK packet
		self.send_packet(&Packet2::default(), &mut write_buf, &mut cobs_buf)
			.expect("Failed to send ACK packet");

		layout
	}

	fn send_packet<P: Packet>(
		&mut self,
		packet: &P,
		write_buf: &mut Vec<u8>,
		cobs_buf: &mut Vec<u8>,
	) -> Result<(), Error> {
		// Serialise packet into bytes
		write_buf.resize(P::max_packet_size(), 0);
		let bytes = packet.serialise(&mut write_buf[..])?;

		// Encode COBS and add null terminator
		cobs_buf.resize(cobs::max_encode_size(bytes.len()), 0);
		let encoded = cobs::encode(bytes, &mut cobs_buf[..])?;

		println!("write {:?}", encoded);

		// Write the packet out
		self.writer.write_all(encoded)?;
		self.writer.write_all(&[0])?;
		self.writer.flush()?;

		Ok(())
	}

	fn recv_msg<'a>(
		&mut self,
		read_buf: &mut Vec<u8>,
		cobs_buf: &'a mut Vec<u8>,
	) -> Result<&'a [u8], Error> {
		// Read the packet in
		read_buf.clear();
		read_buf.reserve(128);
		let bytes_len = self.reader.read_until(0, read_buf)?;
		let bytes = &read_buf[..bytes_len - 1];

		// Decode from COBS
		cobs_buf.resize(cobs::max_decode_size(bytes.len()), 0);
		let decoded = cobs::decode(bytes, &mut cobs_buf[..])?;

		Ok(decoded)
	}

	// Returns the packet on success, on error returns the read data and the error
	// caused
	fn recv_packet<'a, P: Packet>(
		&mut self,
		layout: &Layout,
		read_buf: &mut Vec<u8>,
		cobs_buf: &'a mut Vec<u8>,
	) -> Result<P, Error> {
		let bytes;
		loop {
			// Read and decode cobs message
			let msg = self.recv_msg(read_buf, cobs_buf)?;
			// Read until our message isn't a stream prefix
			if msg != b"sout\n" {
				bytes = msg;
				break;
			}
		}

		match P::deserialise(layout, bytes) {
			Ok(pkt) => Ok(pkt),
			Err(err) => {
				log::debug!(
					"Failed to decode packet ({:?})\nread_buf: {:?}\ncobs_buf: {:?}",
					err,
					read_buf,
					cobs_buf
				);
				Err(err.into())
			}
		}
	}
}

#[derive(Debug)]
enum Error {
	CobsErr(cobs::Error),
	IoErr(io::Error),
	PacketErr(proto::Error),
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

#[derive(Clone)]
pub struct SerialData(Arc<SerialDataInner>);

struct SerialDataInner {
	recv: Mutex<Packet3>,
	send: Mutex<Packet4>,
}

impl SerialData {
	pub fn new(layout: &Layout) -> Self {
		Self(Arc::new(SerialDataInner {
			recv: Mutex::new(Packet3::from_layout(layout)),
			send: Mutex::new(Packet4::from_layout(layout)),
		}))
	}

	pub fn recv_pkt_lock(&self) -> LockResult<MutexGuard<'_, Packet3>> {
		self.0.recv.lock()
	}

	pub fn send_pkt_lock(&self) -> LockResult<MutexGuard<'_, Packet4>> {
		self.0.send.lock()
	}
}

pub fn rtt_test(
	port: &mut Port,
	round_count: usize,
	msg_size: usize,
	msg_byte: u8,
) -> Result<Vec<u32>, String> {
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
		let duration = start.elapsed().as_micros() as _;
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

pub fn rtt_score(times: &[u32]) -> String {
	let mean = (times.iter().map(|t| *t as f32).sum::<f32>() / times.len() as f32) / 1_000.0;
	let values = {
		let mut xs = times.to_vec();
		xs.sort();
		xs.iter().map(|t| *t as f32 / 1_000.0).collect::<Vec<_>>()
	};

	const AMOUNT: usize = 4;
	format!(
		"RTT scores:\n\tmean: {mean}ms\n\tbottom {AMOUNT}: {}...\n\ttop {AMOUNT}: ...{}",
		values[0..AMOUNT]
			.iter()
			.fold(String::new(), |xs, x| format!("{xs}{x} "))
			.trim(),
		values[values.len() - AMOUNT..values.len()]
			.iter()
			.fold(String::new(), |xs, x| format!("{xs}{x} "))
			.trim(),
	)
}
