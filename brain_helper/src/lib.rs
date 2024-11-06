#![no_std]
pub use protocol;

pub use vexide;

extern crate alloc;

use alloc::vec::Vec;

use protocol::*;
const MAX_COBS_PACKET_SIZE: usize = 2047;

// see https://github.com/vexide/vex-sdk/blob/main/src/serial.rs
const SERIAL_CHANNEL_STDIO: u32 = 1;

const COBS_DELIMITER: u8 = 0;

pub fn read_pkt_serial() -> Option<ToBrain> {
    let mut buf = [0u8; MAX_COBS_PACKET_SIZE];

    let mut pkt_len = 0;
    for e in &mut buf.iter_mut() {
        let byte = unsafe { vex_sdk::vexSerialReadChar(SERIAL_CHANNEL_STDIO) };

        // on read fail returns -1 which will early exit
        let byte: u8 = byte.try_into().ok()?;

        *e = byte;

        // COBS delimiter byte read
        if byte == COBS_DELIMITER {
            break;
        }
        pkt_len += 1;
    }

    // no COBS delimiter detected
    if pkt_len == MAX_COBS_PACKET_SIZE {
        return None;
    }

    postcard::from_bytes_cobs(&mut buf).ok()
}

pub fn write_pkt_serial(pkt: ToRobot) -> bool {
    let mut buf = [0u8; MAX_COBS_PACKET_SIZE];

    let Ok(slice) = postcard::to_slice_cobs(&pkt, &mut buf) else {
        return false;
    };

    if unsafe { vex_sdk::vexSerialWriteFree(SERIAL_CHANNEL_STDIO) < slice.len() as i32 } {
        return false;
    }

    unsafe {
        if vex_sdk::vexSerialWriteBuffer(SERIAL_CHANNEL_STDIO, slice.as_ptr(), slice.len() as u32)
            != 1
        {
            return false;
        }
    }

    true
}
