use alloc::{boxed::Box, ffi::CString, string::String, sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};

use pros::{devices::Colour, rtos::Mutex};

pub const SCREEN_WIDTH: i16 = 480;
pub const SCREEN_HEIGHT: i16 = 240;
pub const SCREEN_WIDTH_2: i16 = SCREEN_WIDTH / 2;
pub const SCREEN_HEIGHT_2: i16 = SCREEN_HEIGHT / 2;

#[derive(Copy, Clone, PartialEq)]
pub enum ButtonState {
	Released,
	Pressed,
}

pub struct Button {
	pub centre: (i16, i16),
	pub size: (u16, u16),
	pub button_colour: Colour,
	pub text_colour: Colour,
	pub state: ButtonState,
	pub text: String,
	pub on_press: Option<Box<dyn Fn(&mut Button) + Send + Sync>>,
}

impl Button {
	pub fn intersects(&self, pos: (i16, i16)) -> bool {
		let top = self.centre.1 - self.size.1 as i16;
		let right = self.centre.0 + self.size.0 as i16;
		let bottom = self.centre.1 + self.size.1 as i16;
		let left = self.centre.0 - self.size.0 as i16;

		pos.0 <= right && pos.0 >= left && pos.1 <= top && pos.1 >= bottom
	}

	pub fn update(&mut self, input: InputUpdate) -> bool {
		// Check if we should redraw
		let mut dirty = false;

		// Check if the touch intersects this button
		if !self.intersects(input.pos) {
			return false;
		}

		// Modify the button state
		match (input.touch_status, self.state) {
			(TouchStatus::Released, ButtonState::Pressed) => {
				self.state = ButtonState::Released;
				dirty = true;
			}
			(TouchStatus::Pressed, ButtonState::Released) => {
				// Check if we have an on press callback
				if let Some(f) = self.on_press.take() {
					f(self);
					self.on_press = Some(f);
				}
				self.state = ButtonState::Pressed;
				dirty = true;
			}
			_ => { /* no state changes should have occured */ }
		}

		dirty
	}

	pub fn render(&self) {
		let colour = match self.state {
			ButtonState::Pressed => Colour::new(
				f32::max(self.button_colour.get_r() as f32 * 0.7, 0.1) as _,
				f32::max(self.button_colour.get_g() as f32 * 0.7, 0.1) as _,
				f32::max(self.button_colour.get_b() as f32 * 0.7, 0.1) as _,
			),
			ButtonState::Released => self.button_colour,
		};

		unsafe {
			pros_sys::screen_set_pen(colour.as_u32());
			pros_sys::screen_fill_rect(
				self.centre.0 - self.size.0 as i16,
				self.centre.1 - self.size.1 as i16,
				self.centre.0 + self.size.0 as i16,
				self.centre.1 + self.size.1 as i16,
			);
			let text = CString::new(self.text.clone()).unwrap();
			pros_sys::screen_set_pen(self.text_colour.as_u32());
			pros_sys::screen_print_at(
				pros_sys::text_format_e_t_E_TEXT_MEDIUM_CENTER,
				self.centre.0,
				self.centre.1,
				text.as_bytes_with_nul().as_ptr() as _,
			);
		}
	}
}

impl Default for Button {
	fn default() -> Self {
		Button {
			centre: (SCREEN_WIDTH_2, SCREEN_HEIGHT_2),
			size: (70, 70),
			button_colour: Colour::new(230, 230, 230),
			text_colour: Colour::new(0, 0, 0),
			state: ButtonState::Released,
			text: String::new(),
			on_press: None,
		}
	}
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
			pros_sys::screen_erase();
		}
	}

	pub fn force_render(&self) {
		self.blank();
		for button in &self.buttons {
			button.render();
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

impl From<pros_sys::screen_touch_status_s> for InputUpdate {
	fn from(other: pros_sys::screen_touch_status_s) -> Self {
		Self {
			touch_status: other.touch_status.into(),
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

				for button in &mut screen.buttons {
					if button.update(status) {
						button.render();
					}
				}
			}
			None => {}
		}
	}
}
