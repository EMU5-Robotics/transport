#![no_std]

#[cfg(test)]
extern crate std;
extern crate alloc;

mod packets;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
	BadPacket,
	BufferOverrun,
	InvalidPacketId,
	InvalidValue,
}

pub use crate::packets::*;
