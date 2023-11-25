#![no_std]
#![no_main]

extern crate alloc;
extern crate pros;

use core::sync::atomic::AtomicU8;
use pros::prelude::*;
use screen::*;

mod screen;
mod serial;

static AUTON_PROGRAM: AtomicU8 = AtomicU8::new(0);

struct VexRobot {
	screen: ScreenHandle,
}

impl Robot for VexRobot {
	fn new(_: pros::prelude::Devices) -> Self {
		pros::rtos::tasks::spawn(|| serial::serial_task());

		Self {
			screen: Self::create_screen(),
		}
	}

	fn competition_init(&'static self, _: CompetitionState) {
		self.refresh_screen();
	}

	fn disabled(&'static self, _: CompetitionState) {
		self.refresh_screen();
	}

	fn autonomous(&'static self, _: CompetitionState) {
		self.refresh_screen();
	}

	fn opcontrol(&'static self, _: CompetitionState) {
		self.refresh_screen();
	}
}

impl VexRobot {
	fn refresh_screen(&self) {
		Task::delay(Duration::from_millis(50));
		self.screen.0.lock().force_render();
	}

	fn create_screen() -> ScreenHandle {
		use alloc::{string::String, vec};

		let red1 = {
			let mut btn = Button::default();
			btn.centre = (0 + btn.size.0 as i16, 0 + btn.size.1 as i16);
			btn.button_colour = Colour::new(200, 45, 45);
			btn.text = String::from("RED1");
			btn.id = 1;
			btn
		};

		let red2 = {
			let mut btn = Button::default();
			btn.centre = (0 + btn.size.0 as i16, SCREEN_HEIGHT - btn.size.1 as i16);
			btn.button_colour = Colour::new(200, 45, 45);
			btn.text = String::from("RED2");
			btn.id = 2;
			btn
		};

		let blue1 = {
			let mut btn = Button::default();
			btn.centre = (SCREEN_WIDTH - btn.size.0 as i16, 0 + btn.size.1 as i16);
			btn.button_colour = Colour::new(25, 129, 249);
			btn.text = String::from("BLUE1");
			btn.id = 3;
			btn
		};

		let blue2 = {
			let mut btn = Button::default();
			btn.centre = (
				SCREEN_WIDTH - btn.size.0 as i16,
				SCREEN_HEIGHT - btn.size.1 as i16,
			);
			btn.button_colour = Colour::new(25, 129, 249);
			btn.text = String::from("BLUE2");
			btn.id = 4;
			btn
		};

		let skills1 = {
			let mut btn = Button::default();
			btn.size = (40, 40);
			btn.centre = (SCREEN_WIDTH_2, SCREEN_HEIGHT_2 - (btn.size.1 as i16 + 10));
			btn.button_colour = Colour::new(75, 202, 79);
			btn.text = String::from("SKILLS1");
			btn.id = 5;
			btn
		};

		let skills2 = {
			let mut btn = Button::default();
			btn.size = (40, 40);
			btn.centre = (SCREEN_WIDTH_2, SCREEN_HEIGHT_2 + (btn.size.1 as i16 + 10));
			btn.button_colour = Colour::new(75, 202, 79);
			btn.text = String::from("SKILLS2");
			btn.id = 6;
			btn
		};

		let buttons = vec![red1, red2, blue1, blue2, skills1, skills2];

		// Create and install the screen instance
		Screen {
			buttons,
			clear: Colour::new(30, 30, 30),
		}
		.install_handle()
	}
}

robot!(VexRobot);
