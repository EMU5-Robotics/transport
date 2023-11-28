use alloc::{boxed::Box, sync::Arc};
use core::ptr;

use pros::prelude::*;
use pros::{
	devices::{
		motor::{EncoderUnits, Motor},
		rotation::RotationSensor,
		Direction,
	},
	ports::Port,
};

use protocol::{
	self as proto,
	device::{CompetitionState, ControllerButtons, Gearbox, MotorState, PortState},
	ControlPkt, Devices, ErrorPkt, GenericPkt, InitPkt, Packet, StatusPkt,
};

const MOTOR_TIMEOUT: Duration = Duration::from_millis(50);

pub fn serial_task() {
	let streams = configure_streams();
	let mut reader = UsbSerialReader::new(streams.0);
	let mut writer = UsbSerialWriter::new(streams.1);

	let mut state = State::WaitingForInit;
	let last_packet = Arc::new(Mutex::new(Instant::now()));
	let mut last_plugged = new_devices_plugged([DeviceType::None; 20]).1;

	// Spawn the motor halt task
	{
		let last_packet = last_packet.clone();
		pros::rtos::tasks::spawn(move || loop {
			if last_packet.lock().elapsed() > MOTOR_TIMEOUT {
				// Stop motors
				for i in 1..21 {
					unsafe {
						pros_sys::motor_move(i, 0);
					}
				}
			}
			Task::delay(Duration::from_millis(100));
		});
	}

	loop {
		// Don't be greedy, give some time to other tasks as need
		Task::delay(Duration::from_millis(1));

		match state {
			// Wait for an incoming init packet
			State::WaitingForInit => match reader.recieve::<InitPkt>(&Devices::default()) {
				// If we got an init packet then send out device list out
				Ok(init_pkt) => {
					log::info!("Got InitPkt");
					set_gearboxes(init_pkt.gearboxes).ok();
					state = State::SendDeviceList;
				}
				// Ignore the packet otherwise
				Err(Error::PacketErr(_)) => {
					log::debug!("Got bad packet when expecting InitPkt")
				}
				// If there was another error then it must be fatal
				Err(err) => {
					log::error!("Failed to read packet");
					state = State::FatalError(err);
					continue;
				}
			},
			// Send a list of devices to the host controller
			State::SendDeviceList => {
				let devices = match gather_devices() {
					Ok(x) => x,
					Err(err) => {
						log::error!("Failed to gather devices");
						state = State::FatalError(err.into());
						continue;
					}
				};

				match writer.send(devices.clone()) {
					Ok(_) => {}
					Err(err) => {
						log::error!("Failed to send packet");
						state = State::FatalError(err);
						continue;
					}
				}

				state = State::Operating(devices);
			}
			State::Operating(ref devices) => {
				// Read an incoming packet
				let pkt = match reader.recieve::<GenericPkt>(devices) {
					Ok(x) => x,
					Err(err) => {
						log::error!("Failed to read packet in");
						state = State::FatalError(err);
						continue;
					}
				};

				// Update last packet
				*last_packet.lock() = Instant::now();
				// Check for newly plugged devices
				let (updated, plugged) = new_devices_plugged(last_plugged);
				last_plugged = plugged;
				if updated {
					log::info!("New device plugged in");
					state = State::FailureRecovery;
					continue;
				}

				match pkt {
					// Move the motors, etc.
					GenericPkt::ControlPkt(ctrl) => match sink(ctrl) {
						Ok(_) => {}
						Err(err) => {
							log::warn!("Device error: {:?}", err);
							state = State::FailureRecovery;
							continue;
						}
					},
					// Restart the connection for whatever reason
					GenericPkt::InitPkt(_) => {
						state = State::SendDeviceList;
						continue;
					}
					// Any other packets are invalid and should error
					pkt => {
						log::warn!("Unexpected packet: {:?}", pkt);
						state = State::FailureRecovery;
						continue;
					}
				}

				// Collect information about the devices on the robot
				let pkt = match source(&devices) {
					Ok(x) => x,
					Err(err) => {
						log::warn!("Device error: {:?}", err);
						state = State::FailureRecovery;
						continue;
					}
				};
				// Send it back
				match writer.send(pkt) {
					Ok(_) => {}
					Err(err) => {
						log::error!("Failed to send packet");
						state = State::FatalError(err);
						continue;
					}
				}
			}
			State::FailureRecovery => {
				log::warn!("A recoverable error happened, sending ErrorPkt and new device list");
				// Send an error packet without regard for errors
				writer.send(ErrorPkt::recoverable()).ok();
				state = State::SendDeviceList;
				continue;
			}
			State::FatalError(ref err) => {
				log::error!(
					"Fatal error {:?} occurred falling back to basic driver control",
					err
				);
				// TODO: Determine proper behaviour?
				// For now we will just treat fatal errors as recoverable errors anyway

				// Send an error packet without regard for errors
				writer.send(ErrorPkt::recoverable()).ok();
				state = State::SendDeviceList;
				continue;
			}
		}
	}
}

#[derive(Debug, Clone)]
enum State {
	WaitingForInit,
	SendDeviceList,
	Operating(Devices),
	FailureRecovery,
	FatalError(Error),
}

fn gather_devices() -> Result<Devices, DeviceError> {
	let mut devices = Devices::default();

	// enumerate each port and look for devices
	for port_num in 1..=20 {
		let port = unsafe { Port::new_unchecked(port_num) };
		match port.plugged_type() {
			DeviceType::Motor => {
				let mut motor = port.into_motor_default()?;
				motor.set_encoder_units(EncoderUnits::Ticks)?;
				motor.set_gearing(Gearset::Blue)?;
				motor.tare_position()?;
				devices.set_port(port_num as _, PortState::Motor);
			}
			DeviceType::Rotation => {
				let _ = port
					.into_rotation_sensor(Direction::Forward)?
					.set_data_rate(5)?;
				devices.set_port(port_num as _, PortState::Encoder);
			}
			DeviceType::None => {}
			_ => devices.set_port(port_num as _, PortState::Other),
		}
	}

	Ok(devices)
}

fn new_devices_plugged(last: [DeviceType; 20]) -> (bool, [DeviceType; 20]) {
	let mut plugged = [DeviceType::None; 20];
	for port_num in 1..=20 {
		let port = unsafe { Port::new_unchecked(port_num) };
		plugged[(port_num - 1) as usize] = port.plugged_type();
	}

	(last != plugged, plugged)
}

fn set_gearboxes(gearboxes: [Gearbox; 20]) -> Result<(), DeviceError> {
	for (port, gearbox) in gearboxes.iter().enumerate() {
		let port = unsafe { Port::new_unchecked(port as u8 + 1) };
		match port.plugged_type() {
			DeviceType::Motor => {
				let mut motor = Motor { port };
				match gearbox {
					Gearbox::Blue => motor.set_gearing(Gearset::Blue)?,
					Gearbox::Green => motor.set_gearing(Gearset::Green)?,
					Gearbox::Red => motor.set_gearing(Gearset::Red)?,
				}
			}
			_ => {}
		}
	}
	Ok(())
}

// ptrs aren't send + sync safe by default, this specific type is
#[derive(Clone, Copy)]
struct CFile(*mut libc::FILE);

unsafe impl Send for CFile {}
unsafe impl Sync for CFile {}

impl core::ops::Deref for CFile {
	type Target = *mut libc::FILE;

	fn deref(&self) -> &Self::Target {
		&self.0
	}
}

fn configure_streams() -> (CFile, CFile) {
	unsafe {
		// disable cobs, we do it manually
		pros_sys::serctl(15, ptr::null_mut());
		// enable sin stream
		pros_sys::serctl(10, 0x706e6973 as *mut libc::c_void);
		// enable sout stream
		pros_sys::serctl(10, 0x74756f73 as *mut libc::c_void);

		let mode = pros::cstr!("r+");
		let sin_fs = pros::cstr!("/ser/sin");
		let sout_fs = pros::cstr!("/ser/sout");

		let sin = libc::fopen(sin_fs, mode);
		let sout = libc::fopen(sout_fs, mode);
		if sin.is_null() || sout.is_null() {
			panic!("failed to open input/output streams");
		}

		(CFile(sin), CFile(sout))
	}
}

#[derive(Debug, Clone)]
enum Error {
	PacketErr(proto::Error),
	CobsErr(cobs::Error),
	DeviceErr(DeviceError),
	BufferOverrun,
	Io,
}

impl From<proto::Error> for Error {
	fn from(x: proto::Error) -> Self {
		match x {
			proto::Error::BufferOverrun => Self::BufferOverrun,
			x => Self::PacketErr(x),
		}
	}
}

impl From<cobs::Error> for Error {
	fn from(x: cobs::Error) -> Self {
		match x {
			cobs::Error::OutputBufferFull => Self::BufferOverrun,
			x => Self::CobsErr(x),
		}
	}
}

impl From<DeviceError> for Error {
	fn from(x: DeviceError) -> Self {
		Self::DeviceErr(x)
	}
}

const BUFFER_SIZE: usize = 256;

struct UsbSerialReader {
	stdin: CFile,
	read_buf: Box<[u8]>,
	cobs_buf: Box<[u8]>,
}

impl UsbSerialReader {
	pub fn new(stdin: CFile) -> Self {
		Self {
			stdin,
			read_buf: alloc::vec![0; BUFFER_SIZE].into(),
			cobs_buf: alloc::vec![0; BUFFER_SIZE].into(),
		}
	}

	fn read<'a>(&'a mut self) -> Result<&'a [u8], (&'a [u8], Error)> {
		// Read the packet in
		let mut i = 0;
		let read = loop {
			// Read byte and check for EOF
			let c = unsafe {
				let c = libc::fgetc(*self.stdin);
				if libc::feof(*self.stdin) != 0 {
					return Err((&self.read_buf[..i], Error::Io));
				}
				c
			};

			// Check if was null byte
			if c == 0 {
				break &self.read_buf[..i];
			} else {
				self.read_buf[i] = c as _;
				i += 1;
			}

			// Check space is sufficient
			if i > self.read_buf.len() {
				return Err((&self.read_buf[..i - 1], Error::Io));
			}
		};

		// Decode from COBS
		let msg = cobs::decode(read, &mut self.cobs_buf).map_err(|e| ((read, e.into())))?;

		Ok(msg)
	}

	pub fn recieve<'a, P: Packet>(&'a mut self, devices: &Devices) -> Result<P, Error> {
		// Read until we get a message that isn't the stream header
		let bytes;
		loop {
			let msg = self.read().map_err(|(_, err)| err)?;
			if msg != b"sinp\n" {
				bytes = msg;
				break;
			}
		}

		let pkt = P::deserialise(devices, bytes)?;
		Ok(pkt)
	}
}

struct UsbSerialWriter {
	stdout: CFile,
	cobs_buf: Box<[u8]>,
	write_buf: Box<[u8]>,
}

impl UsbSerialWriter {
	pub fn new(stdout: CFile) -> Self {
		Self {
			stdout,
			cobs_buf: alloc::vec![0; BUFFER_SIZE].into(),
			write_buf: alloc::vec![0; BUFFER_SIZE].into(),
		}
	}

	pub fn write<'a>(&'a mut self, msg: &[u8]) -> Result<(), Error> {
		let msg = cobs::encode(msg, &mut self.cobs_buf)?;

		unsafe {
			if libc::fwrite(msg.as_ptr() as _, 1, msg.len(), *self.stdout) == 0 {
				return Err(Error::Io);
			}
			if libc::fputc(0, *self.stdout) < 0 {
				return Err(Error::Io);
			}
		}

		Ok(())
	}

	pub fn send<P: Packet>(&mut self, pkt: P) -> Result<(), Error> {
		let mut buf = core::mem::take(&mut self.write_buf);
		let bytes = pkt.serialise(&mut buf)?;
		self.write(bytes)?;
		let _ = core::mem::replace(&mut self.write_buf, buf);
		Ok(())
	}
}

fn source(devices: &Devices) -> Result<StatusPkt, Error> {
	let mut pkt = StatusPkt::from_devices(devices);

	// Competition state
	pkt.state = CompetitionState::from_bits_truncate(CompetitionMode::get_status().bits());

	// Controller state
	let controller = unsafe { Controller::master() };
	pkt.controller_axes = [
		controller.get_analog_raw(Axis::LeftX)?,
		controller.get_analog_raw(Axis::LeftY)?,
		controller.get_analog_raw(Axis::RightX)?,
		controller.get_analog_raw(Axis::RightY)?,
	];
	pkt.controller_buttons =
		ControllerButtons::from_bits_truncate(controller.get_buttons()?.bits());

	// Encoder values
	for encoder in devices.iter(PortState::is_rotation) {
		let device = unsafe {
			RotationSensor {
				port: Port::new_unchecked(encoder as _),
			}
		};
		pkt.set_encoder(encoder, device.get_position()?);
	}

	// Motor encoders and state
	for motor in devices.iter(PortState::is_motor) {
		let device = unsafe {
			Motor {
				port: Port::new_unchecked(motor as _),
			}
		};

		let state = MotorState::new(
			device.get_current_draw()?,
			device.get_voltage()?,
			device.get_temperature()? as _,
			device.get_actual_velocity()? as _,
		);

		pkt.set_encoder(motor, (device.get_position()? * 100.0) as _);
		pkt.set_motor_state(motor, state);
	}

	// Current autonomous selection
	pkt.auton = crate::AUTON_PROGRAM.load(core::sync::atomic::Ordering::Acquire);

	Ok(pkt)
}

fn sink(pkt: ControlPkt) -> Result<(), DeviceError> {
	// Set the motor voltages
	for (port, power, is_velocity) in pkt.motor_powers() {
		let mut motor = Motor {
			port: unsafe { Port::new_unchecked(port) },
		};
		if is_velocity {
			motor.move_velocity(power as _)?;
		} else {
			motor.move_voltage(power)?;
		}
	}
	// Set the triport pins
	for i in 0..8 {
		let active = pkt.triport_pins >> i & 0x01 == 1;
		let triport = unsafe { TriPort::new_unchecked(i + 1, None).into_digital_out().ok() };
		if let Some(mut port) = triport {
			port.write(active);
		}
	}
	Ok(())
}
