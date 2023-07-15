#![no_std]
#![no_main]

extern crate alloc;
extern crate pros;

use alloc::{ffi::CString, format, sync::Arc};
use core::ptr;

use pros::prelude::*;
use pros::{
	devices::{motor::Motor, rotation::RotationSensor as Encoder, Direction},
	ports::{DeviceType, Port},
	rtos::tasks::TaskBuilder,
};

use protocol::{self as proto, Layout, Packet, Packet1, Packet2, Packet3, Packet4};

const BUFFER_SIZE: usize = 256;

#[allow(dead_code)]
struct VexRobot {
	serial_writer_task: Task,
	serial_reader_task: Task,
	serial_watchdog: Task,
}

impl VexRobot {
	fn gather_devices() -> (Vec<Motor>, Vec<Encoder>) {
		let mut motors = Vec::new();
		let mut encoders = Vec::new();

		// enumerate each port and look for devices
		unsafe {
			for port in 1..=21 {
				let port = Port::new_unchecked(port);
				match port.plugged_type() {
					DeviceType::Motor => {
						motors.push(port.into_motor_default().unwrap());
					}
					DeviceType::Rotation => {
						encoders.push(port.into_rotation_sensor(Direction::Forward).unwrap());
					}
					_ => { /* do nothing */ }
				}
			}
		}

		(motors, encoders)
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

	fn serial_handshake(
		motors: &[Motor],
		encoders: &[Encoder],
		(sin, sout): (CFile, CFile),
	) -> Layout {
		// Create packet1/layout
		let pkt1 = {
			let mut pkt = Packet1::default();
			pkt.triports = 0xFF; // just assume all are connect, no guarantee
			for motor in motors {
				pkt.ports[(motor.get_port() - 1) as usize] = proto::PortState::Motor;
			}
			for encoder in encoders {
				pkt.ports[(encoder.get_port() - 1) as usize] = proto::PortState::Encoder;
			}
			pkt
		};

		let mut write_buf = Vec::with_capacity(BUFFER_SIZE);
		let mut read_buf = Vec::with_capacity(BUFFER_SIZE);
		let mut cobs_buf = Vec::with_capacity(BUFFER_SIZE);
		unsafe {
			write_buf.set_len(BUFFER_SIZE);
			read_buf.set_len(BUFFER_SIZE);
			cobs_buf.set_len(BUFFER_SIZE);
		}

		send_packet(sout, &pkt1, &mut write_buf, &mut cobs_buf).expect("Failed to send Packet1");
		pros::screen_print_at(1, pros::cstr!("send pkt"));
		_ = recv_packet::<Packet2>(sin, &pkt1, &mut read_buf, &mut cobs_buf)
			.expect("Failed to receive Packet2");
		pros::screen_print_at(3, pros::cstr!("recv pkt"));

		pkt1
	}

	fn serial_writer_task(
		watchdog: WatchdogNotifier,
		sout: CFile,
		motors: Vec<Motor>,
		encoders: Vec<Encoder>,
		layout: Layout,
	) {
		let mut write_buf = Vec::with_capacity(BUFFER_SIZE);
		let mut cobs_buf = Vec::with_capacity(BUFFER_SIZE);
		unsafe {
			write_buf.set_len(BUFFER_SIZE);
			cobs_buf.set_len(BUFFER_SIZE);
		}
		let base_pkt = Packet3::from_layout(&layout);

		let mut f = || -> Result<(), Error> {
			let mut pkt = base_pkt.clone();

			// TODO: competition state
			pkt.state = proto::CompetitionState::User;

			// Controller state
			unsafe {
				let controller = Controller::master();
				let axes = [
					controller.get_analog_raw(Axis::LeftX)?,
					controller.get_analog_raw(Axis::LeftY)?,
					controller.get_analog_raw(Axis::RightX)?,
					controller.get_analog_raw(Axis::RightY)?,
				];
				let buttons =
					proto::ControllerButtons::from_bits_truncate(controller.get_buttons()?.bits());

				pkt.controller_buttons = buttons;
				pkt.controller_axes = axes;
			}
			// Encoder values
			for encoder in &encoders {
				let value = encoder.get_position()?;
				pkt.set_encoder(encoder.get_port() as _, value);
			}
			// Motor encoders and state
			for motor in &motors {
				let port = motor.get_port() as _;

				let position = (motor.get_position()? * 100.0) as i32;
				pkt.set_encoder(port, position);

				let state = proto::MotorState::new(
					motor.get_current_draw()?,
					motor.get_voltage()?,
					motor.get_temperature()? as _,
				);
				pkt.set_motor_state(port, state);
			}

			_ = send_packet(sout, &pkt, &mut write_buf, &mut cobs_buf)?;

			Ok(())
		};

		let mut timer = Interval::new(Duration::from_millis(5));
		loop {
			// Write message and handle errors
			match f() {
				Ok(_) => watchdog.lock().last_write = Instant::now(),
				Err(err) => {
					let mut watchdog = watchdog.lock();
					watchdog.writer_error = Some(err);
					return;
				}
			}

			// Check if the other half is dead
			{
				let watchdog = watchdog.lock();
				if watchdog.reader_error.is_some() {
					return;
				}
			}

			timer.delay();
		}
	}

	fn serial_reader_task(watchdog: WatchdogNotifier, sin: CFile, layout: Layout) {
		let mut read_buf = Vec::with_capacity(BUFFER_SIZE);
		let mut cobs_buf = Vec::with_capacity(BUFFER_SIZE);
		unsafe {
			read_buf.set_len(BUFFER_SIZE);
			cobs_buf.set_len(BUFFER_SIZE);
		}

		let mut f = || -> Result<(), (Vec<u8>, Error)> {
			let pkt = recv_packet::<Packet4>(sin, &layout, &mut read_buf, &mut cobs_buf)
				.map_err(|(d, e)| (d.into(), e.into()))?;

			// Set the motor voltages
			for (port, power) in pkt.motor_power.iter().enumerate() {
				let power = match power {
					Some(p) => p,
					None => continue,
				};
				let mut motor = Motor {
					port: unsafe { Port::new_unchecked(port as _) },
				};
				motor
					.move_voltage(*power)
					.map_err(|e| (Vec::new(), e.into()))?;
			}

			// Set the triport toggles
			for i in 0..8 {
				let active = pkt.triport_toggle >> i & 0x01 == 1;
				let mut triport = unsafe { TriPort::new_unchecked(i + 1, None) }
					.into_digital_out()
					.map_err(|e| (Vec::new(), e.into()))?;
				triport.write(active);
			}

			Ok(())
		};

		let mut timer = Interval::new(Duration::from_millis(5));
		loop {
			// Read messages and handle errors
			match f() {
				Ok(_) => watchdog.lock().last_read = Instant::now(),
				Err((data, err)) => {
					let mut watchdog = watchdog.lock();
					watchdog.reader_error = Some((data.into(), err));
					return;
				}
			}

			// Check if the other half is dead
			{
				let watchdog = watchdog.lock();
				if watchdog.writer_error.is_some() {
					return;
				}
			}

			timer.delay();
		}
	}

	fn serial_watchdog(reader: Task, writer: Task, watchdog_notifier: WatchdogNotifier) {
		const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(1200);

		let mut timer = Interval::new(Duration::from_millis(20));
		loop {
			let mut notifier = watchdog_notifier.lock();
			if notifier.last_read.elapsed() > WATCHDOG_TIMEOUT {
				notifier.reader_error = Some((Vec::new(), Error::Timeout));
				pros::screen_print_at(1, pros::cstr!("killed reader"));
				reader.clone().delete();
			}
			if notifier.last_write.elapsed() > WATCHDOG_TIMEOUT {
				notifier.writer_error = Some(Error::Timeout);
				pros::screen_print_at(2, pros::cstr!("killed writer"));
				writer.clone().delete();
			}

			timer.delay();
		}
	}
}

impl Robot for VexRobot {
	fn new(_: Devices) -> Self {
		Task::delay(Duration::from_millis(500));

		// Get resources
		let (motors, encoders) = Self::gather_devices();
		let streams = Self::configure_streams();

		pros::screen_print_at(0, pros::cstr!("starting handshake"));
		// Perform the initial handshake
		let layout = Self::serial_handshake(&motors, &encoders, streams);
		pros::screen_print_at(4, pros::cstr!("testing1"));

		// Create watchdog resource
		let watchdog_notifier = WatchdogNotifier::new();

		let serial_writer_task = TaskBuilder::new()
			.name("coprocessor-writerd".into())
			.priority(Task::PRIORITY_DEFAULT + 1)
			.spawn({
				let layout = layout.clone();
				let watchdog = watchdog_notifier.clone();
				move || Self::serial_writer_task(watchdog, streams.1, motors, encoders, layout)
			})
			.unwrap();

		let serial_reader_task = TaskBuilder::new()
			.name("coprocessor-readerd".into())
			.priority(Task::PRIORITY_DEFAULT + 1)
			.spawn({
				let layout = layout.clone();
				let watchdog = watchdog_notifier.clone();
				move || Self::serial_reader_task(watchdog, streams.0, layout)
			})
			.unwrap();

		let serial_watchdog = TaskBuilder::new()
			.name("coprocessor-watchdog".into())
			.priority(Task::PRIORITY_DEFAULT - 1)
			.spawn({
				let reader = serial_reader_task.clone();
				let writer = serial_writer_task.clone();
				|| Self::serial_watchdog(reader, writer, watchdog_notifier)
			})
			.unwrap();

		Self {
			serial_writer_task,
			serial_reader_task,
			serial_watchdog,
		}
	}
}

robot!(VexRobot);

#[derive(Clone)]
struct WatchdogNotifier(Arc<Mutex<WatchdogNotifierInner>>);

impl core::ops::Deref for WatchdogNotifier {
	type Target = Arc<Mutex<WatchdogNotifierInner>>;

	fn deref(&self) -> &Self::Target {
		&self.0
	}
}

impl WatchdogNotifier {
	pub fn new() -> Self {
		let inner = WatchdogNotifierInner {
			last_write: Instant::now(),
			last_read: Instant::now(),
			writer_error: None,
			reader_error: None,
		};

		Self(Arc::new(Mutex::new(inner)))
	}
}

struct WatchdogNotifierInner {
	last_write: Instant,
	last_read: Instant,
	writer_error: Option<Error>,
	reader_error: Option<(Vec<u8>, Error)>,
}

#[derive(Debug)]
enum Error {
	PacketErr(proto::Error),
	CobsErr(cobs::Error),
	DeviceErr(DeviceError),
	BufferOverrun,
	Io,
	Timeout,
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

fn send_packet<P: Packet>(
	stream: CFile,
	packet: &P,
	write_buf: &mut [u8],
	cobs_buf: &mut [u8],
) -> Result<(), Error> {
	let bytes = packet.serialise(write_buf)?;
	let encoded = cobs::encode(bytes, cobs_buf)?;

	unsafe {
		if libc::fwrite(encoded.as_ptr() as _, 1, encoded.len(), *stream) == 0 {
			return Err(Error::Io);
		}
		if libc::fputc(0, *stream) < 0 {
			return Err(Error::Io);
		}
	}

	Ok(())
}

extern "C" {
	pub fn dev_ctl(
		arg: *const core::ffi::c_void,
		cmd: u32,
		extra_arg: *const core::ffi::c_void,
	) -> core::ffi::c_int;
}

// Returns the packet on success, on error returns the read data and the error
// caused
fn recv_packet<'a, P: Packet>(
	stream: CFile,
	layout: &Layout,
	read_buf: &'a mut [u8],
	cobs_buf: &mut [u8],
) -> Result<P, (&'a [u8], Error)> {
	// Read the packet in
	let mut i = 0;
	let read = loop {
		// is there data?
		if unsafe { dev_ctl(*stream as _, 16, ptr::null_mut()) } != 0 {
			pros::screen_print_at(2, pros::cstr!("data ready"));
		} else {
			pros::screen_print_at(2, pros::cstr!("no data ready"));
		}

		// Read byte and check for EOF
		let c = unsafe {
			let c = libc::fgetc(*stream);
			if libc::feof(*stream) != 0 {
				return Err((&read_buf[..i], Error::Io));
			}
			c
		};

		pros::screen_print_at(
			2,
			CString::new(format!("read byte {}: {}", i, c))
				.unwrap()
				.as_c_str()
				.as_ptr() as _,
		);

		// Check if was null byte
		if c == 0 {
			break &read_buf[..i];
		} else {
			read_buf[i] = c as _;
			i += 1;
		}

		// Check space is sufficient
		if i > read_buf.len() {
			return Err((&read_buf[..i - 1], Error::Io));
		}
	};
	// Decode from COBS
	let decoded = cobs::decode(read, cobs_buf).map_err(|e| ((read, e.into())))?;
	// Deserialise packet
	let pkt = P::deserialise(layout, decoded).map_err(|e| ((read, e.into())))?;

	Ok(pkt)
}
