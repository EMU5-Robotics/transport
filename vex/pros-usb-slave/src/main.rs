#![no_std]
#![no_main]

extern crate alloc;
extern crate pros;

use core::sync::atomic::{AtomicU8, Ordering};
use pros::prelude::*;
use screen::*;

mod screen;
mod serial;

static AUTON_PROGRAM: AtomicU8 = AtomicU8::new(0);

struct VexRobot {
	#[allow(dead_code)]
	screen: ScreenHandle,
}

impl Robot for VexRobot {
	fn new(_: pros::prelude::Devices) -> Self {
		pros::rtos::tasks::spawn(|| serial::serial_task());

		Self {
			screen: Self::create_screen(),
		}
	}
}

impl VexRobot {
	fn create_screen() -> ScreenHandle {
		use alloc::{boxed::Box, string::String, vec};

		let red1 = {
			let mut btn = Button::default();
			btn.centre = (0 + btn.size.1 as i16 / 2, 0 + btn.size.1 as i16 / 2); // Top left justify
			btn.button_colour = Colour::new(200, 45, 45);
			btn.text = String::from("RED1");
			btn.on_press = Some(Box::new(|_| AUTON_PROGRAM.store(1, Ordering::Release)));
			btn
		};

		let red2 = {
			let mut btn = Button::default();
			btn.centre = (
				0 + btn.size.1 as i16 / 2,
				SCREEN_HEIGHT - btn.size.1 as i16 / 2,
			); // Bottom left justify
			btn.button_colour = Colour::new(200, 45, 45);
			btn.text = String::from("RED2");
			btn.on_press = Some(Box::new(|_| AUTON_PROGRAM.store(2, Ordering::Release)));
			btn
		};

		let blue1 = {
			let mut btn = Button::default();
			btn.centre = (
				SCREEN_WIDTH - btn.size.1 as i16 / 2,
				0 + btn.size.1 as i16 / 2,
			); // Top right justify
			btn.button_colour = Colour::new(25, 129, 249);
			btn.text = String::from("BLUE1");
			btn.on_press = Some(Box::new(|_| AUTON_PROGRAM.store(3, Ordering::Release)));
			btn
		};

		let blue2 = {
			let mut btn = Button::default();
			btn.centre = (
				SCREEN_WIDTH - btn.size.1 as i16 / 2,
				SCREEN_HEIGHT - btn.size.1 as i16 / 2,
			); // Bottom right justify
			btn.button_colour = Colour::new(25, 129, 249);
			btn.text = String::from("BLUE2");
			btn.on_press = Some(Box::new(|_| AUTON_PROGRAM.store(4, Ordering::Release)));
			btn
		};

		let buttons = vec![red1, red2, blue1, blue2];

		// Create and install the screen instance
		let screen = Screen {
			buttons,
			clear: Colour::new(70, 70, 70),
		}
		.install_handle();
		// Force a single initial render of the screen
		screen.0.lock().force_render();
		screen
	}
}

robot!(VexRobot);
