#![no_std]

use core::convert::TryFrom;
use core::mem::size_of;
use num_enum::{IntoPrimitive, TryFromPrimitive};

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod test;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
	BufferOverrun,
	BadPacket,
	InvalidValue,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
pub enum PortState {
	Motor = 1,
	Encoder,
	Unplugged = 0xFF,
}

impl PortState {
	pub fn is_encoder(self) -> bool {
		matches!(self, Self::Motor | Self::Encoder)
	}

	pub fn is_motor(self) -> bool {
		matches!(self, Self::Motor)
	}
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
pub enum CompetitionState {
	Initialise = 1,
	Disabled,
	CompetitionInitialise,
	Autonomous,
	User,
	Unknown = 0xFF,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct MotorState {
	current: u32,
	voltage: i32,
	temperature: i32,
}

impl MotorState {
	pub fn new(current: u32, voltage: i32, temperature: f32) -> Self {
		Self {
			current,
			voltage,
			temperature: (temperature * 10.0) as i32,
		}
	}

	pub fn quantise(&self) -> (u8, i8, i8) {
		(
			(self.current as f32 / (2_500.0 / 255.0)).clamp(0.0, 255.0) as _,
			(self.voltage as f32 / (12_000.0 / 127.0)).clamp(-127.0, 127.0) as _,
			(self.temperature as f32 / 700.0).clamp(-127.0, 127.0) as _,
		)
	}

	pub fn quantise_inverse(values: (u8, i8, i8)) -> Self {
		Self {
			current: (values.0 as f32 * (2_500.0 / 255.0)) as _,
			voltage: (values.1 as f32 * (12_000.0 / 127.0)) as _,
			temperature: (values.2 as f32 * 700.0) as _,
		}
	}
}

// This should match the bitflags as defined within pros-rs
bitflags::bitflags! {
	#[derive(Debug, Clone, Copy, PartialEq, Eq)]
	pub struct ControllerButtons: u16 {
		const A = 0b0000_0000_0001;
		const B = 0b0000_0000_0010;
		const X = 0b0000_0000_0100;
		const Y = 0b0000_0000_1000;
		const UP = 0b0000_0001_0000;
		const DOWN = 0b0000_0010_0000;
		const LEFT = 0b0000_0100_0000;
		const RIGHT = 0b0000_1000_0000;
		const L1 = 0b0001_0000_0000;
		const L2 = 0b0010_0000_0000;
		const R1 = 0b0100_0000_0000;
		const R2 = 0b1000_0000_0000;
	}
}

pub trait Packet: Sized {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error>;
	fn deserialise(layout: &Layout, bytes: &[u8]) -> Result<Self, Error>;
	fn max_packet_size() -> usize;
}

pub type Layout = Packet1;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet1 {
	pub ports: [PortState; 20],
	pub triports: u8,
}

impl Default for Packet1 {
	fn default() -> Self {
		Self {
			ports: [PortState::Unplugged; 20],
			triports: 0,
		}
	}
}

impl Packet1 {
	pub fn motor_count(&self) -> usize {
		self.ports
			.iter()
			.filter(|p| PortState::is_motor(**p))
			.count()
	}

	pub fn encoder_count(&self) -> usize {
		self.ports
			.iter()
			.filter(|p| PortState::is_encoder(**p))
			.count()
	}
}

impl Packet for Packet1 {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out = BufWriter::new(buffer, Error::BufferOverrun);

		for port in self.ports {
			out = out.append(&Into::<u8>::into(port).to_be_bytes())?;
		}
		out = out.append(&self.triports.to_be_bytes())?;

		Ok(out.finish())
	}

	fn deserialise(_: &Layout, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::max_packet_size() {
			return Err(Error::BadPacket);
		}

		let mut pkt = Self::default();
		for i in 0..20 {
			pkt.ports[i] = PortState::try_from(bytes[i]).map_err(|_| Error::InvalidValue)?;
		}
		pkt.triports = bytes[20];

		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		size_of::<PortState>() * 20 + size_of::<u8>() * 1
	}
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet2 {
	ack: u8,
}

impl Default for Packet2 {
	fn default() -> Self {
		Self { ack: 0xAA }
	}
}

impl Packet for Packet2 {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let out = BufWriter::new(buffer, Error::BufferOverrun)
			.append(&self.ack.to_be_bytes())?
			.finish();

		Ok(out)
	}

	fn deserialise(_: &Layout, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::max_packet_size() {
			return Err(Error::BadPacket);
		}

		if bytes[0] != 0xAA {
			return Err(Error::InvalidValue);
		}

		Ok(Self::default())
	}

	fn max_packet_size() -> usize {
		1
	}
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet3 {
	pub state: CompetitionState,
	pub controller_buttons: ControllerButtons,
	/// lx, ly, rx, ry
	pub controller_axes: [i8; 4],
	pub encoders: [Option<i32>; 20],
	pub motor_state: [Option<MotorState>; 20],
}

impl Default for Packet3 {
	fn default() -> Self {
		Self {
			state: CompetitionState::Unknown,
			controller_buttons: ControllerButtons::empty(),
			controller_axes: [0; 4],
			encoders: [None; 20],
			motor_state: [None; 20],
		}
	}
}

impl Packet3 {
	pub fn from_layout(layout: &Layout) -> Self {
		let mut pkt = Self::default();

		for (i, port) in layout.ports.iter().enumerate() {
			if port.is_encoder() {
				pkt.set_encoder(i, 0);
				if port.is_motor() {
					pkt.set_motor_state(i, MotorState::default());
				}
			}
		}

		pkt
	}

	pub fn set_motor_state(&mut self, motor: usize, state: MotorState) {
		self.motor_state[motor] = Some(state);
	}

	pub fn set_encoder(&mut self, port: usize, value: i32) {
		self.encoders[port] = Some(value);
	}

	const fn packet_size(encoders: usize, motors: usize) -> usize {
		1 + 6 + size_of::<i32>() * encoders + (size_of::<i8>() * 3) * motors
	}
}

impl Packet for Packet3 {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out = BufWriter::new(buffer, Error::BufferOverrun);

		out = out.append(&Into::<u8>::into(self.state).to_be_bytes())?;
		out = out.append(&self.controller_buttons.bits().to_be_bytes())?;
		for axis in self.controller_axes {
			out = out.append(&axis.to_be_bytes())?;
		}
		for ticks in self.encoders.iter().filter_map(|p| *p) {
			out = out.append(&ticks.to_be_bytes())?;
		}
		for state in self.motor_state.iter().filter_map(|p| *p) {
			let s = state.quantise();
			out = out
				.append(&s.0.to_be_bytes())?
				.append(&s.1.to_be_bytes())?
				.append(&s.2.to_be_bytes())?;
		}

		Ok(out.finish())
	}

	fn deserialise(layout: &Layout, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::packet_size(layout.encoder_count(), layout.motor_count()) {
			return Err(Error::BadPacket);
		}

		let mut i = 0;

		let mut pkt = Self::default();
		pkt.state = CompetitionState::try_from(bytes[i]).map_err(|_| Error::InvalidValue)?;
		i += 1;

		pkt.controller_buttons =
			ControllerButtons::from_bits(u16::from_be_bytes(bytes[i..i + 2].try_into().unwrap()))
				.ok_or(Error::InvalidValue)?;
		i += 2;

		for j in 0..4 {
			pkt.controller_axes[j] = i8::from_be_bytes(bytes[i + j..i + j + 1].try_into().unwrap());
		}
		i += 4;

		for (port, _) in layout
			.ports
			.iter()
			.enumerate()
			.filter(|(_, p)| p.is_encoder())
		{
			let value = i32::from_be_bytes(bytes[i..i + 4].try_into().unwrap());
			pkt.set_encoder(port, value);

			i += 4;
		}

		for (port, _) in layout
			.ports
			.iter()
			.enumerate()
			.filter(|(_, p)| p.is_motor())
		{
			let v1 = u8::from_be_bytes(bytes[i..i + 1].try_into().unwrap());
			i += 1;
			let v2 = i8::from_be_bytes(bytes[i..i + 1].try_into().unwrap());
			i += 1;
			let v3 = i8::from_be_bytes(bytes[i..i + 1].try_into().unwrap());
			i += 1;

			pkt.set_motor_state(port, MotorState::quantise_inverse((v1, v2, v3)));
		}

		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		Self::packet_size(20, 20)
	}
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet4 {
	pub motor_power: [Option<i16>; 20],
	pub triport_toggle: u8,
}

impl Default for Packet4 {
	fn default() -> Self {
		Self {
			motor_power: [None; 20],
			triport_toggle: 0,
		}
	}
}

impl Packet4 {
	pub fn from_layout(layout: &Layout) -> Self {
		let mut pkt = Self::default();

		for (i, port) in layout.ports.iter().enumerate() {
			if port.is_motor() {
				pkt.set_motor(i, 0);
			}
		}

		pkt
	}

	pub fn set_motor(&mut self, motor: usize, power: i16) {
		self.motor_power[motor] = Some(power);
	}

	const fn packet_size(motors: usize) -> usize {
		size_of::<i16>() * motors + size_of::<u8>() * 1
	}
}

impl Packet for Packet4 {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out = BufWriter::new(buffer, Error::BufferOverrun);

		for power in self.motor_power.iter().filter_map(|p| *p) {
			out = out.append(&power.to_be_bytes())?;
		}
		out = out.append(&self.triport_toggle.to_be_bytes())?;

		Ok(out.finish())
	}

	fn deserialise(layout: &Layout, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::packet_size(layout.motor_count()) {
			return Err(Error::BadPacket);
		}

		let mut i = 0;
		let mut pkt = Self::default();

		for (port, _) in layout
			.ports
			.iter()
			.enumerate()
			.filter(|(_, p)| p.is_motor())
		{
			let value = i16::from_be_bytes(bytes[i..i + size_of::<i16>()].try_into().unwrap());
			pkt.set_motor(port, value);

			i += size_of::<i16>();
		}

		pkt.triport_toggle = u8::from_be_bytes(bytes[i..i + size_of::<u8>()].try_into().unwrap());

		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		Self::packet_size(20)
	}
}

#[derive(Debug)]
struct BufWriter<'a, E: Copy> {
	buffer: &'a mut [u8],
	i: usize,
	error: E,
}

impl<'a, E: Copy> BufWriter<'a, E> {
	pub fn new(buffer: &'a mut [u8], error: E) -> Self {
		Self {
			buffer,
			i: 0,
			error,
		}
	}

	pub fn append(mut self, slice: &[u8]) -> Result<Self, E> {
		let len = self.i + slice.len();
		if len > self.buffer.len() {
			return Err(self.error);
		}
		self.buffer[self.i..len].copy_from_slice(slice);
		self.i = len;

		Ok(self)
	}

	pub fn finish(self) -> &'a [u8] {
		&self.buffer[0..self.i]
	}
}
