#![no_std]

extern crate alloc;
#[cfg(test)]
extern crate std;

mod packets;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
	BadPacket,
	BufferOverrun,
	InvalidPacketId,
	InvalidValue,
}

pub use crate::packets::*;
