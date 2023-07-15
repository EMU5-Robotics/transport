use std::{collections::BTreeMap, string::String, vec::Vec};

pub const BROADCAST_PORT: u16 = 17746;
pub const SERVER_PORT: u16 = 17747;
pub const BROADCAST_MSG: &[u8] = b"VEX RS BROADCAST SERVER";

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Role {
	Robot,
	Client,
	Server, // serializes as nothing, this for when ID == 0
}

pub type ClientId = u8;
pub const MAX_CLIENT_ID: ClientId = 2 ^ 7;
pub const SERVER_CLIENT_ID: ClientId = 0;

// rust representation of the message format
pub struct Message {
	pub id: ClientId,
	pub role: Role,
	pub timestamp: u32,
	pub msg: MessageContent,
}

pub type MessageTypeSize = u8;

// de/serialised either by hand or with metaprogramming
pub enum MessageContent {
	ClientInit {
		name: Option<String>,
		role: Role,
	},
	ClientInitAck {
		given_id: ClientId,
		name: String,
		server_last_timestamp: u32,
	},
	DataRequest {
		// id of the source we want data to be sent from
		data_source_id: ClientId,
		// the type of data that is being requested
		msg_ty: MessageTypeSize,
		// how often the data is requested
		period: RequestPeriod,
	},
	DataRequestAck {
		success: bool,
	},
	// request from the server using DataRequest
	PeersList(Vec<(ClientId, Role, String)>),

	// general types of data packets being sent //

	// every motor connected
	MotorData(BTreeMap<u8, MotorData>),
	// command packets //
}

// how often data has been requested to be sent
pub enum RequestPeriod {
	Once,
	WhenUpdated,
}

pub struct MotorData {
	pub temp: i16,
	pub current: i16,
	pub voltage: i16,
	pub rotations: i32,
}

impl Message {
	pub fn parse(_bytes: &[u8]) -> Result<Message, ()> {
		unimplemented!()
	}

	pub fn encode(&self, _buffer: &mut Vec<u8>) {
		unimplemented!()
	}
}
