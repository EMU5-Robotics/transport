#![no_std]
#![no_main]

extern crate alloc;
extern crate pros;

use pros::prelude::*;

mod serial;

struct VexRobot {}

impl Robot for VexRobot {
	fn new(_: pros::prelude::Devices) -> Self {
		pros::rtos::tasks::spawn(|| Self::serial_task());

		Self {}
	}
}

robot!(VexRobot);
