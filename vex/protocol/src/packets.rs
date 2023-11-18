use crate::Error;
use core::{convert::TryFrom, mem::size_of};

use num_enum::{IntoPrimitive, TryFromPrimitive};

/// This type is used to define the layout of packets. It allows deserialisation
/// to determine port/device the next packet will be.
pub type Devices = DevicesPkt;

pub trait Packet: Sized {
	/// Serialise a packet into the provided buffer. Bounds checking must be
	/// performed and invariant must be check before serialisation. The very first
	/// byte provided must be the packet ID.
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error>;
	/// Deserialise a packet from the provided bytes. Expects a packet ID at the
	/// front.
	fn deserialise(devices: &Devices, bytes: &[u8]) -> Result<Self, Error>;
	fn max_packet_size() -> usize;
	fn packet_id() -> u8;
}

use device::*;
pub mod device {
	use num_enum::{IntoPrimitive, TryFromPrimitive};

	#[repr(u8)]
	#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
	pub enum PortState {
		Motor = 1,
		Encoder,
		Other = 0xFE,
		Unplugged = 0xFF,
	}

	bitflags::bitflags! {
		/// Bitflags for defining the state of the robot in competition mode.
		#[derive(Debug, Clone, Copy, PartialEq, Eq)]
		pub struct CompetitionState: u8 {
			/// The brain is disabled.
			const DISABLED = 0x1 << 0x0;
			/// The brain is in autonomous mode.
			const AUTONOMOUS = 0x1 << 0x1;
			/// The brain is connected to the competition control.
			const CONNECTED = 0x1 << 0x2;
		}
	}

	#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
	pub struct MotorState {
		pub current: u32,
		pub voltage: i32,
		pub temperature: i32,
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

	#[repr(u8)]
	#[derive(Debug, Clone, Copy, PartialEq, Eq)]
	pub enum ControllerAxis {
		LeftX,
		LeftY,
		RightX,
		RightY,
	}

	impl PortState {
		pub fn is_rotation(self) -> bool {
			matches!(self, Self::Encoder)
		}

		pub fn is_encoder(self) -> bool {
			matches!(self, Self::Motor | Self::Encoder)
		}

		pub fn is_motor(self) -> bool {
			matches!(self, Self::Motor)
		}
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
}

/// An enum that can contain any packet type and can generically deserialise a
/// packet into this enum.
#[derive(Debug, Clone)]
pub enum GenericPkt {
	InitPkt(InitPkt),
	DevicesPkt(DevicesPkt),
	StatusPkt(StatusPkt),
	ControlPkt(ControlPkt),
	ErrorPkt(ErrorPkt),
}

impl Packet for GenericPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		match self {
			Self::InitPkt(pkt) => pkt.serialise(buffer),
			Self::DevicesPkt(pkt) => pkt.serialise(buffer),
			Self::StatusPkt(pkt) => pkt.serialise(buffer),
			Self::ControlPkt(pkt) => pkt.serialise(buffer),
			Self::ErrorPkt(pkt) => pkt.serialise(buffer),
		}
	}

	/// This function will try every possible packet using the prefixed ID. If
	/// no valid packet ID is found then this function will return [`Error::BadPacket`].
	fn deserialise(devices: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.is_empty() {
			return Err(Error::BadPacket);
		}

		match bytes[0] {
			x if InitPkt::packet_id() == x => {
				Ok(Self::InitPkt(InitPkt::deserialise(devices, bytes)?))
			}
			x if DevicesPkt::packet_id() == x => {
				Ok(Self::DevicesPkt(DevicesPkt::deserialise(devices, bytes)?))
			}
			x if StatusPkt::packet_id() == x => {
				Ok(Self::StatusPkt(StatusPkt::deserialise(devices, bytes)?))
			}
			x if ControlPkt::packet_id() == x => {
				Ok(Self::ControlPkt(ControlPkt::deserialise(devices, bytes)?))
			}
			x if ErrorPkt::packet_id() == x => {
				Ok(Self::ErrorPkt(ErrorPkt::deserialise(devices, bytes)?))
			}
			_ => Err(Error::BadPacket),
		}
	}

	fn max_packet_size() -> usize {
		panic!("generic packet has no pre-defined max size");
	}

	fn packet_id() -> u8 {
		panic!("generic packet has no pre-defined ID");
	}
}

impl Default for GenericPkt {
	fn default() -> Self {
		Self::ControlPkt(ControlPkt::default())
	}
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InitPkt {
	ack: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DevicesPkt {
	// Zero indexed array of each port that is connected the brain currently.
	ports: [PortState; 20],
	// Bitmap for which triports are known connected. LSB marks the 1st port.
	// This will in practice always likely be `0xFF`
	pub triports: u8,

	// Memorised counts of the number of motors and encoders
	motor_count: u8,
	encoder_count: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusPkt {
	/// The current state of the competition as reported by the VEX Brain.
	pub state: CompetitionState,
	/// A bitmap containing all the currently pressed buttons.
	pub controller_buttons: ControllerButtons,
	/// Indexed as LX, LY, RX, RY. You can use [`ControllerAxis`] to index with a
	/// meaningful name.
	pub controller_axes: [i8; 4],
	/// The position of each encoder that is present
	encoder_positions: [Option<i32>; 20],
	/// The state of each motor that is present
	motor_states: [Option<MotorState>; 20],
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ControlPkt {
	// The power for the motors
	powers: [Option<i16>; 20],
	/// Whether each triport pin should be set to HIGH or LOW
	pub triport_pins: u8,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoPrimitive, TryFromPrimitive)]
pub enum ErrorType {
	Recoverable = 0x55,
	Fatal = 0xCC,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ErrorPkt {
	pub err: ErrorType,
}

impl Default for InitPkt {
	fn default() -> Self {
		Self { ack: 0xAA }
	}
}

impl Default for DevicesPkt {
	fn default() -> Self {
		Self {
			ports: [PortState::Unplugged; 20],
			triports: 0xFF,
			motor_count: 0,
			encoder_count: 0,
		}
	}
}

impl Default for StatusPkt {
	fn default() -> Self {
		Self {
			state: CompetitionState::empty(),
			controller_buttons: ControllerButtons::empty(),
			controller_axes: [0; 4],
			encoder_positions: [None; 20],
			motor_states: [None; 20],
		}
	}
}

impl Default for ErrorPkt {
	fn default() -> Self {
		Self {
			err: ErrorType::Recoverable,
		}
	}
}

impl InitPkt {
	const PKT_ID: u8 = 0x20;
}

impl DevicesPkt {
	const PKT_ID: u8 = 0x21;

	pub fn set_port(&mut self, port: usize, state: PortState) {
		assert!(port >= 1 && port <= 20);
		self.ports[port - 1] = state;

		self.motor_count = self
			.ports
			.iter()
			.filter(|p| PortState::is_motor(**p))
			.count() as _;
		self.encoder_count = self
			.ports
			.iter()
			.filter(|p| PortState::is_encoder(**p))
			.count() as _;
	}

	pub fn get_port(&self, port: usize) -> PortState {
		assert!(port >= 1 && port <= 20);
		self.ports[port - 1]
	}

	pub fn motor_count(&self) -> usize {
		self.motor_count as _
	}

	pub fn encoder_count(&self) -> usize {
		self.encoder_count as _
	}

	pub fn iter(&self, f: fn(PortState) -> bool) -> impl Iterator<Item = usize> + '_ {
		self.ports
			.iter()
			.enumerate()
			.filter(move |(_, p)| f(**p))
			.map(|(p, _)| p + 1)
	}
}

impl StatusPkt {
	const PKT_ID: u8 = 0x23;

	pub fn from_devices(devices: &Devices) -> Self {
		let mut pkt = Self::default();

		for (i, port) in devices.ports.iter().enumerate().map(|(i, p)| (i + 1, p)) {
			if port.is_encoder() {
				pkt.set_encoder(i, 0);
				if port.is_motor() {
					pkt.set_motor_state(i, MotorState::default());
				}
			}
		}

		pkt
	}

	pub fn set_motor_state(&mut self, port: usize, state: MotorState) {
		assert!(port >= 1 && port <= 20);
		self.motor_states[port - 1] = Some(state);
	}

	pub fn get_motor_state(&self, port: usize) -> Option<MotorState> {
		assert!(port >= 1 && port <= 20);
		self.motor_states[port - 1]
	}

	pub fn motors(&self) -> impl Iterator<Item = (u8, MotorState)> + '_ {
		self.motor_states
			.iter()
			.enumerate()
			.filter_map(|(i, &s)| Some((i as u8, s?)))
	}

	pub fn set_encoder(&mut self, port: usize, value: i32) {
		assert!(port >= 1 && port <= 20);
		self.encoder_positions[port - 1] = Some(value);
	}

	pub fn get_encoder(&self, port: usize) -> Option<i32> {
		assert!(port >= 1 && port <= 20);
		self.encoder_positions[port - 1]
	}

	pub fn encoders(&self) -> impl Iterator<Item = (u8, i32)> + '_ {
		self.encoder_positions
			.iter()
			.enumerate()
			.filter_map(|(i, &s)| Some((i as u8, s?)))
	}

	const fn packet_size(encoders: usize, motors: usize) -> usize {
		1 + 1 + 6 + size_of::<i32>() * encoders + (size_of::<i8>() * 3) * motors
	}
}

impl ControlPkt {
	const PKT_ID: u8 = 0x24;

	pub fn from_devices(devices: &Devices) -> Self {
		let mut pkt = Self::default();

		for (i, port) in devices.ports.iter().enumerate() {
			if port.is_motor() {
				pkt.set_power(i + 1, 0);
			}
		}

		pkt
	}

	pub fn motor_powers(&self) -> impl Iterator<Item = (u8, i16)> + '_ {
		self.powers
			.iter()
			.enumerate()
			.filter(|(_, p)| p.is_some())
			.map(|(i, p)| ((i + 1) as u8, p.unwrap()))
	}

	pub fn set_power(&mut self, port: usize, power: i16) {
		assert!(port >= 1 && port <= 20);
		self.powers[port - 1] = Some(power);
	}

	const fn packet_size(motors: usize) -> usize {
		1 + size_of::<i16>() * motors + size_of::<u8>() * 1
	}
}

impl ErrorPkt {
	const PKT_ID: u8 = 0x29;

	pub const fn recoverable() -> Self {
		Self {
			err: ErrorType::Recoverable,
		}
	}

	pub const fn fatal() -> Self {
		Self {
			err: ErrorType::Fatal,
		}
	}
}

impl Packet for InitPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let out = BufWriter::new(buffer, Error::BufferOverrun)
			.append(Self::packet_id().to_be_bytes())?
			.append(self.ack.to_be_bytes())?
			.finish();

		Ok(out)
	}

	fn deserialise(_: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::max_packet_size() {
			return Err(Error::BadPacket);
		}
		let mut bytes = BufReader::new(bytes);
		if bytes.read_u8() != Self::packet_id() {
			return Err(Error::InvalidPacketId);
		}

		if bytes.read_u8() != 0xAA {
			return Err(Error::InvalidValue);
		}

		debug_assert!(bytes.is_empty());
		Ok(Self::default())
	}

	fn max_packet_size() -> usize {
		1 + 1
	}

	fn packet_id() -> u8 {
		Self::PKT_ID
	}
}

impl Packet for DevicesPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out =
			BufWriter::new(buffer, Error::BufferOverrun).append(Self::packet_id().to_be_bytes())?;

		for port in self.ports {
			out = out.append((port as u8).to_be_bytes())?;
		}
		out = out.append(self.triports.to_be_bytes())?;

		Ok(out.finish())
	}

	fn deserialise(_: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::max_packet_size() {
			return Err(Error::BadPacket);
		}
		let mut bytes = BufReader::new(bytes);
		if bytes.read_u8() != Self::packet_id() {
			return Err(Error::InvalidPacketId);
		}

		let mut pkt = Self::default();
		for i in 1..=20 {
			let port = PortState::try_from(bytes.read_u8()).map_err(|_| Error::InvalidValue)?;
			pkt.set_port(i, port)
		}
		pkt.triports = bytes.read_u8();

		debug_assert!(bytes.is_empty());
		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		1 + size_of::<PortState>() * 20 + size_of::<u8>() * 1
	}

	fn packet_id() -> u8 {
		Self::PKT_ID
	}
}

impl Packet for StatusPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out =
			BufWriter::new(buffer, Error::BufferOverrun).append(Self::packet_id().to_be_bytes())?;

		out = out.append(self.state.bits().to_be_bytes())?;
		out = out.append(self.controller_buttons.bits().to_be_bytes())?;
		for axis in self.controller_axes {
			out = out.append(axis.to_be_bytes())?;
		}
		for ticks in self.encoder_positions.iter().filter_map(|p| *p) {
			out = out.append(ticks.to_be_bytes())?;
		}
		for state in self.motor_states.iter().filter_map(|p| *p) {
			let s = state.quantise();
			out = out
				.append(s.0.to_be_bytes())?
				.append(s.1.to_be_bytes())?
				.append(s.2.to_be_bytes())?;
		}

		Ok(out.finish())
	}

	fn deserialise(devices: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::packet_size(devices.encoder_count(), devices.motor_count()) {
			return Err(Error::BadPacket);
		}
		let mut bytes = BufReader::new(bytes);
		if bytes.read_u8() != Self::packet_id() {
			return Err(Error::InvalidPacketId);
		}

		let mut pkt = Self::default();
		pkt.state = CompetitionState::from_bits(bytes.read_u8()).ok_or(Error::InvalidValue)?;

		pkt.controller_buttons =
			ControllerButtons::from_bits(bytes.read_u16()).ok_or(Error::InvalidValue)?;

		for i in 0..4 {
			pkt.controller_axes[i] = bytes.read_i8();
		}

		for port in devices.iter(PortState::is_encoder) {
			pkt.set_encoder(port, bytes.read_i32());
		}

		for port in devices.iter(PortState::is_motor) {
			let v1 = bytes.read_u8();
			let v2 = bytes.read_i8();
			let v3 = bytes.read_i8();
			pkt.set_motor_state(port, MotorState::quantise_inverse((v1, v2, v3)));
		}

		debug_assert!(bytes.is_empty());
		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		Self::packet_size(20, 20)
	}

	fn packet_id() -> u8 {
		Self::PKT_ID
	}
}

impl Packet for ControlPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let mut out =
			BufWriter::new(buffer, Error::BufferOverrun).append(Self::packet_id().to_be_bytes())?;

		for power in self.powers.iter().filter_map(|p| *p) {
			out = out.append(power.to_be_bytes())?;
		}
		out = out.append(self.triport_pins.to_be_bytes())?;

		Ok(out.finish())
	}

	fn deserialise(devices: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::packet_size(devices.motor_count()) {
			return Err(Error::BadPacket);
		}
		let mut bytes = BufReader::new(bytes);
		if bytes.read_u8() != Self::packet_id() {
			return Err(Error::InvalidPacketId);
		}

		let mut pkt = Self::default();
		for port in devices.iter(PortState::is_motor) {
			pkt.set_power(port, bytes.read_i16());
		}

		pkt.triport_pins = bytes.read_u8();

		debug_assert!(bytes.is_empty());
		Ok(pkt)
	}

	fn max_packet_size() -> usize {
		Self::packet_size(20)
	}

	fn packet_id() -> u8 {
		Self::PKT_ID
	}
}

impl Packet for ErrorPkt {
	fn serialise<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], Error> {
		let out = BufWriter::new(buffer, Error::BufferOverrun)
			.append(Self::packet_id().to_be_bytes())?
			.append(Into::<u8>::into(self.err).to_be_bytes())?
			.finish();

		Ok(out)
	}

	fn deserialise(_: &Devices, bytes: &[u8]) -> Result<Self, Error> {
		if bytes.len() != Self::max_packet_size() {
			return Err(Error::BadPacket);
		}
		let mut bytes = BufReader::new(bytes);
		if bytes.read_u8() != Self::packet_id() {
			return Err(Error::InvalidPacketId);
		}

		let err = ErrorType::try_from(bytes.read_u8()).map_err(|_| Error::InvalidValue)?;

		debug_assert!(bytes.is_empty());
		Ok(Self { err })
	}

	fn max_packet_size() -> usize {
		1 + 1
	}

	fn packet_id() -> u8 {
		Self::PKT_ID
	}
}

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

	pub fn append<S: AsRef<[u8]>>(mut self, slice: S) -> Result<Self, E> {
		let slice = slice.as_ref();

		// Check for overrun
		let len = self.i + slice.len();
		if len > self.buffer.len() {
			return Err(self.error);
		}

		// Copy into buffer
		self.buffer[self.i..len].copy_from_slice(slice);
		self.i = len;

		Ok(self)
	}

	pub fn finish(self) -> &'a [u8] {
		&self.buffer[0..self.i]
	}
}

struct BufReader<'a> {
	buffer: &'a [u8],
	i: usize,
}

impl<'a> BufReader<'a> {
	pub fn new(buffer: &'a [u8]) -> Self {
		Self { buffer, i: 0 }
	}

	pub fn read_u8(&mut self) -> u8 {
		let x = u8::from_be_bytes(self.buffer[self.i..self.i + 1].try_into().unwrap());
		self.i += 1;
		x
	}

	pub fn read_i8(&mut self) -> i8 {
		let x = i8::from_be_bytes(self.buffer[self.i..self.i + 1].try_into().unwrap());
		self.i += 1;
		x
	}

	pub fn read_u16(&mut self) -> u16 {
		let x = u16::from_be_bytes(self.buffer[self.i..self.i + 2].try_into().unwrap());
		self.i += 2;
		x
	}

	pub fn read_i16(&mut self) -> i16 {
		let x = i16::from_be_bytes(self.buffer[self.i..self.i + 2].try_into().unwrap());
		self.i += 2;
		x
	}

	pub fn read_i32(&mut self) -> i32 {
		let x = i32::from_be_bytes(self.buffer[self.i..self.i + 4].try_into().unwrap());
		self.i += 4;
		x
	}

	pub fn is_empty(&self) -> bool {
		self.buffer[self.i..].is_empty()
	}
}
