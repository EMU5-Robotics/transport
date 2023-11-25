use alloc::{ffi::CString, string::String, sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};

use pros::{devices::Colour, rtos::Mutex};

pub const SCREEN_WIDTH: i16 = 480 - 1;
pub const SCREEN_HEIGHT: i16 = 240 - 1;
pub const SCREEN_WIDTH_2: i16 = SCREEN_WIDTH / 2;
pub const SCREEN_HEIGHT_2: i16 = SCREEN_HEIGHT / 2;

#[derive(Copy, Clone, PartialEq)]
pub enum ButtonState {
	Released,
	Pressed,
	Toggled,
	ToggledPressed,
}

pub struct Button {
	pub centre: (i16, i16),
	pub size: (u16, u16),
	pub button_colour: Colour,
	pub text_colour: Colour,
	pub state: ButtonState,
	pub text: String,
	pub id: u8,
}

impl Button {
	pub fn intersects(&self, pos: (i16, i16)) -> bool {
		let top = self.centre.1 - self.size.1 as i16;
		let right = self.centre.0 + self.size.0 as i16;
		let bottom = self.centre.1 + self.size.1 as i16;
		let left = self.centre.0 - self.size.0 as i16;

		pos.0 <= right && pos.0 >= left && pos.1 >= top && pos.1 <= bottom
	}

	pub fn update(&mut self, input: InputUpdate) -> (bool, bool) {
		// Check if we should redraw
		let mut dirty = false;
		let mut reset_buttons = false;

		// Check if the touch intersects this button
		if !self.intersects(input.pos) && !matches!(input.touch_status, TouchStatus::Released) {
			return (false, false);
		}

		// Modify the button state
		match (self.state, input.touch_status) {
			(ButtonState::Released, TouchStatus::Pressed) => {
				self.state = ButtonState::Pressed;
				dirty = true;
			}
			(ButtonState::Pressed, TouchStatus::Released) => {
				self.state = ButtonState::Toggled;
				crate::AUTON_PROGRAM.store(self.id, Ordering::Release);
				dirty = true;
				reset_buttons = true;
			}
			(ButtonState::Toggled, TouchStatus::Pressed) => {
				self.state = ButtonState::ToggledPressed;
				dirty = true;
			}
			(ButtonState::ToggledPressed, TouchStatus::Released) => {
				self.state = ButtonState::Released;
				crate::AUTON_PROGRAM.store(0, Ordering::Release);
				dirty = true;
			}
			_ => { /* no state changes should have occured */ }
		}

		(dirty, reset_buttons)
	}

	pub fn render(&self) {
		let colour = match self.state {
			ButtonState::Released => self.button_colour,
			ButtonState::Pressed | ButtonState::ToggledPressed => {
				colour_fade(self.button_colour, 0.5)
			}
			ButtonState::Toggled => colour_fade(self.button_colour, 0.7),
		};

		unsafe {
			pros_sys::screen_set_pen(colour.as_u32());
			pros_sys::screen_fill_rect(
				self.centre.0 - self.size.0 as i16,
				self.centre.1 - self.size.1 as i16,
				self.centre.0 + self.size.0 as i16,
				self.centre.1 + self.size.1 as i16,
			);
			if matches!(
				self.state,
				ButtonState::Toggled | ButtonState::ToggledPressed
			) {
				pros_sys::screen_set_pen(Colour::WHITE.as_u32());
				pros_sys::screen_draw_rect(
					self.centre.0 - self.size.0 as i16,
					self.centre.1 - self.size.1 as i16,
					self.centre.0 + self.size.0 as i16,
					self.centre.1 + self.size.1 as i16,
				);
			}
			let text = CString::new(self.text.clone()).unwrap();
			pros_sys::screen_set_pen(self.text_colour.as_u32());
			pros_sys::screen_set_eraser(colour.as_u32());
			pros_sys::screen_print_at(
				pros_sys::text_format_e_t_E_TEXT_MEDIUM_CENTER,
				self.centre.0 - 5 * self.text.len() as i16,
				self.centre.1 - 7,
				text.as_bytes_with_nul().as_ptr() as _,
			);
		}
	}

	pub fn update_id(&mut self) {
		let id = crate::AUTON_PROGRAM.load(Ordering::Acquire);
		if id != self.id {
			self.state = ButtonState::Released;
		}
	}
}

impl Default for Button {
	fn default() -> Self {
		Button {
			centre: (SCREEN_WIDTH_2, SCREEN_HEIGHT_2),
			size: (50, 50),
			button_colour: Colour::new(230, 230, 230),
			text_colour: Colour::WHITE,
			state: ButtonState::Released,
			text: String::new(),
			id: 0,
		}
	}
}

fn colour_fade(col: Colour, fact: f32) -> Colour {
	Colour::new(
		f32::max(col.get_r() as f32 * fact, 0.1) as _,
		f32::max(col.get_g() as f32 * fact, 0.1) as _,
		f32::max(col.get_b() as f32 * fact, 0.1) as _,
	)
}

pub struct Screen {
	pub buttons: Vec<Button>,
	pub clear: Colour,
}

#[derive(Clone)]
pub struct ScreenHandle(pub Arc<Mutex<Screen>>);

impl Screen {
	fn blank(&self) {
		unsafe {
			pros_sys::screen_set_pen(self.clear.as_u32());
			pros_sys::screen_fill_rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
		}
	}

	pub fn force_render(&self) {
		self.blank();
		for button in &self.buttons {
			button.render();
		}
	}

	pub fn update(&mut self, status: InputUpdate) {
		let mut do_reset = false;
		for button in &mut self.buttons {
			let (redraw, reset) = button.update(status);
			do_reset |= reset;
			if !do_reset && redraw {
				button.render();
			}
		}

		if do_reset {
			for button in &mut self.buttons {
				button.update_id();
				button.render();
			}
		}
	}
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchStatus {
	Released,
	Pressed,
	Held,
	Error,
}

#[derive(Debug, Clone, Copy)]
pub struct InputUpdate {
	pub touch_status: TouchStatus,
	pub pos: (i16, i16),
	pub press_count: u32,
	pub release_count: u32,
}

impl From<pros_sys::last_touch_e_t> for TouchStatus {
	fn from(other: pros_sys::last_touch_e_t) -> Self {
		match other {
			pros_sys::last_touch_e_t_E_TOUCH_RELEASED => TouchStatus::Released,
			pros_sys::last_touch_e_t_E_TOUCH_PRESSED => TouchStatus::Pressed,
			pros_sys::last_touch_e_t_E_TOUCH_HELD => TouchStatus::Held,
			_ => TouchStatus::Error,
		}
	}
}

impl From<pros_sys::screen_touch_status_s_t> for InputUpdate {
	fn from(other: pros_sys::screen_touch_status_s_t) -> Self {
		Self {
			touch_status: (other.touch_status as pros_sys::last_touch_e_t).into(),
			pos: (other.x as _, other.y as _),
			press_count: other.press_count as _,
			release_count: other.release_count as _,
		}
	}
}

static mut SCREEN: Option<ScreenHandle> = None;

static STATE: AtomicUsize = AtomicUsize::new(0);

const UNINITIALISED: usize = 0;
const INITIALISING: usize = 1;
const INITIALISED: usize = 2;

impl Screen {
	/// This function installs the screen callback handlers, it should only be called once in the
	/// programs lifetime
	pub fn install_handle(self) -> ScreenHandle {
		let handle = ScreenHandle(Arc::new(Mutex::new(self)));

		// This is inspire by what the log crate does
		let old_state = match STATE.compare_exchange(
			UNINITIALISED,
			INITIALISING,
			Ordering::SeqCst,
			Ordering::SeqCst,
		) {
			Ok(s) | Err(s) => s,
		};
		match old_state {
			UNINITIALISED => {
				unsafe {
					SCREEN = Some(handle.clone());
				}
				STATE.store(INITIALISED, Ordering::SeqCst);
			}
			_ => panic!("Screen handle already installed"),
		}

		// Set the screen touch callbacks
		unsafe {
			let cb = Some(Self::touch_cb as _);
			pros_sys::screen_touch_callback(cb, pros_sys::last_touch_e_t_E_TOUCH_RELEASED);
			pros_sys::screen_touch_callback(cb, pros_sys::last_touch_e_t_E_TOUCH_PRESSED);
			pros_sys::screen_touch_callback(cb, pros_sys::last_touch_e_t_E_TOUCH_HELD);
		}

		handle
	}

	unsafe extern "C" fn touch_cb() {
		match &SCREEN.as_ref() {
			Some(ref handle) => {
				let mut screen = handle.0.lock();
				let status: InputUpdate = unsafe { pros_sys::screen_touch_status().into() };
				screen.update(status);
			}
			None => {}
		}
	}
}
