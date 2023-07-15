#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;
use core::{iter, ptr};

use pros::prelude::*;

struct VexRobot;

impl Robot for VexRobot {
	fn new(_device: Devices) -> Self {
		unsafe {
			// disable cobs, we do it manually
			pros_sys::serctl(15, ptr::null_mut());
			// enable sin stream
			pros_sys::serctl(10, 0x706e6973 as *mut libc::c_void);
			// enable sout stream
			pros_sys::serctl(10, 0x74756f73 as *mut libc::c_void);
		}
		VexRobot
	}

	fn opcontrol(&'static self, _state: CompetitionState) {
		let (sin, sout) = unsafe {
			let sin = libc::fopen(pros::cstr!("/ser/sin"), pros::cstr!("r+"));
			let sout = libc::fopen(pros::cstr!("/ser/sout"), pros::cstr!("r+"));
			if sin.is_null() || sout.is_null() {
				panic!("failed to open input/output streams");
			}
			(sin, sout)
		};

		let mut input_buf = vec![0u8; 1024];
		let mut decode_buf = vec![0u8; 1024];
		let mut encode_buf = vec![0u8; 1024];

		loop {
			input_buf.clear();

			// read input
			loop {
				unsafe {
					// this is the easiest way more or less to read bytes, seems like it will
					// be more than fast enough, this will block until input is avaliable
					let c = libc::fgetc(sin);
					if libc::feof(sin) != 0 {
						panic!("failed to read input stream");
					}
					input_buf.push(c as _);
					// we have read our entire message input
					if c == 0 {
						break;
					}
				}
			}

			// decode input
			decode_buf.clear();
			decode_buf.extend(iter::repeat(0).take(decode_buf.capacity()));

			let input = cobs::decode(&input_buf[..input_buf.len() - 1], &mut decode_buf).unwrap();

			// encode output
			encode_buf.clear();
			encode_buf.extend(iter::repeat(0).take(encode_buf.capacity()));

			let output = cobs::encode(&input, &mut encode_buf).unwrap();

			// write output
			for b in output {
				unsafe {
					if libc::fputc(*b as _, sout) < 0 {
						panic!("failed to write output")
					}
				}
			}
			unsafe {
				if libc::fputc(0, sout) < 0 {
					panic!("failed to write output")
				}
			}
		}
	}
}

robot!(VexRobot);
