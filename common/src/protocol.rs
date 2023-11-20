use std::{string::String, vec::Vec};

use anyhow::Result;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Packet {
	pub source: ClientId,
	pub target: ClientId,
	pub timestamp: u32,
	pub msg: ControlMessage,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum ControlMessage {
	/// Connect to the server as `role`. Can provide a requested name.
	ClientInit {
		name: Option<String>,
		role: Role,
	},
	/// Returned by the server when a connection is established. The given ID to be used by the
	/// client is provided.
	ClientInitAck {
		given_id: ClientId,
		name: String,
		server_last_timestamp: u32,
	},
	/// Request a list of peers (other clients and robots)
	PeerListRequest,
	/// Response to [`ControlMessage::PeerListRequest`], a list of the client ID and the name
	/// of the connection.
	PeerList(Vec<(ClientId, String)>),
	DataRequest {
		/// The type of data being requested. Make sure that this is a request message.
		data: DataMessage,
		/// How often this data is requested.
		period: RequestPeriod,
	},
	DataResponse {
		/// The requested data as expected by [`ControlMessage::DataRequest`], returns `None` if
		/// the data source doesn't exist or another error happened.
		data: Option<DataMessage>,
	},
	/// A message to managed source on a robot. Used for program upload, etc.
	ManageMessage(ManageMessage),
	/// A wrapper for when an invalid Message is sent, will be returned to sender.
	InvalidMessage(Box<Packet>),
	UnknownClientId(ClientId),
}

/// how often data has been requested to be sent
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum RequestPeriod {
	/// Send the data only once after it was requested.
	Once,
	/// Send the data every time a change is made (be wary for noisy sources),
	/// prefer [`RequestPeriod::AtMost`].
	WhenUpdated,
	/// Send the data continuously when updated at most every x milliseconds.
	AtMost(u16),
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum DataMessage {
	/* Request Messages */
	GetRobotLocation,
	/* Response Messages */
	RobotLocation { pos: (f32, f32), heading: f32 },
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum ManageMessage {
	/* Request Messages */
	/// Upload a new program to the robot under a specific name. This will overwrite any previous
	/// programs with the same name. Has no response.
	UploadProgram { name: String, data: Vec<u8> },
	/// List all programs on the robot. Response is [`ManageMessage::ProgramList`].
	ListPrograms,
	/// Delete a specific program on the robot as listed from [`ManageMessage::ProgramList`]. Has
	/// no response.
	DeleteProgram(String),
	/// The name of a program to run as listed from [`ManageMessage::ProgramList`]. Has no
	/// response.
	RunProgram(String),
	/// Announce rerun server location,
	AnnounceRerunServer(std::net::SocketAddr),

	/* Response Messages */
	/// A list of program on the robot. Requested by [`ManageMessage::ListPrograms`].
	ProgramList(Vec<String>),
}

impl Packet {
	#[inline]
	pub fn encode(&self, buf: &mut Vec<u8>) {
		// `Write` doesn't fail for `Vec<u8>`
		ciborium::into_writer(self, buf).unwrap();
	}

	#[inline]
	pub fn decode(buf: &[u8]) -> Result<Self> {
		let this = ciborium::from_reader(buf)?;
		Ok(this)
	}
}

/* this below is a stupid ClientId I start with for a proper binary format, might come back
 * to this later on. But for now just using it anyway in conjunction with the CBOR format.
 */

use packed_struct::{prelude::*, PackingResult};

/// What role the identified participant in the conversation takes.
#[derive(PrimitiveEnum, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Role {
	Client = 0,
	Robot = 1,
	Server = 2,
}

/// A unique 7-bit ID for a Robot/Client along with a [`Role`].
#[derive(PackedStruct, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[packed_struct(bit_numbering = "msb0", size_bytes = "1")]
pub struct ClientId {
	#[packed_field(bits = "0..=6")]
	id: Integer<u8, packed_bits::Bits<7>>,
	#[packed_field(bits = "7", ty = "enum")]
	role: Role,
}

impl ClientId {
	/// The maximum possible ID something can have. The total number of assignable IDs is
	/// `MAX_ID - 1`.
	pub const MAX_ID: u8 = (1 << 7) - 1;

	/// Create a new [`ClientId`], checks that if the role is Server, the ID is 0.
	#[inline]
	pub fn new(id: u8, role: Role) -> Self {
		// ID must be 0 iff Role is server
		if role == Role::Server || id == 0 {
			assert!(role == Role::Server && id == 0);
		}

		Self {
			id: id.into(),
			role,
		}
	}

	/// Get the [`ClientId`] for the server
	#[inline]
	pub fn server() -> Self {
		Self::new(0, Role::Server)
	}

	/// This is just the same as the server ID
	#[inline]
	pub fn null() -> Self {
		Self::new(0, Role::Server)
	}

	#[inline]
	pub fn unpack(src: &<Self as PackedStruct>::ByteArray) -> PackingResult<Self> {
		let id = <Self as PackedStruct>::unpack(&src)?;
		if *id.id == 0 {
			Ok(Self::server())
		} else {
			Ok(id)
		}
	}

	#[inline]
	pub fn unpack_from_slice(src: &[u8]) -> PackingResult<Self> {
		let id = <Self as PackedStructSlice>::unpack_from_slice(src)?;
		if *id.id == 0 {
			Ok(Self::server())
		} else {
			Ok(id)
		}
	}

	#[inline]
	pub fn id(&self) -> u8 {
		*self.id
	}

	#[inline]
	pub fn role(&self) -> Role {
		self.role
	}

	pub fn to_string(&self) -> String {
		match self.role {
			Role::Client => format!("{}C", self.id),
			Role::Robot => format!("{}R", self.id),
			Role::Server => format!("null"),
		}
	}
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ParseClientIdError {
	NoTypeMarker,
	UnknownCharacter,
	ParseNumberError,
}

impl std::str::FromStr for ClientId {
	type Err = ParseClientIdError;

	fn from_str(s: &str) -> Result<Self, Self::Err> {
		if s == "null" || s == "NULL" {
			return Ok(ClientId::null());
		}
		let marker_idx = match s.find(|c: char| !c.is_ascii_digit()) {
			Some(idx) => idx,
			None => return Err(ParseClientIdError::NoTypeMarker),
		};
		let id = match s[0..marker_idx].parse::<u8>() {
			Ok(id) => id,
			Err(_) => return Err(ParseClientIdError::ParseNumberError),
		};
		let role = match &s[marker_idx..marker_idx + 1] {
			"C" | "c" => Role::Client,
			"R" | "r" => Role::Robot,
			_ => return Err(ParseClientIdError::UnknownCharacter),
		};
		if marker_idx + 1 < s.len() {
			return Err(ParseClientIdError::UnknownCharacter);
		}

		Ok(ClientId::new(id, role))
	}
}

#[cfg(test)]
mod tests {
	#[test]
	fn clientid() {
		let robot = ClientId::new(1, Role::Robot);
		let client = ClientId::new(2, Role::Client);
		let server = ClientId::server();

		let packed: [u8; 1] = robot.pack().unwrap();
		assert_eq!(packed, [0b1000_0001]);
		let unpacked = ClientId::unpack_smart(&packed).unwrap();
		assert_eq!(unpacked, robot);

		let packed: [u8; 1] = client.pack().unwrap();
		assert_eq!(packed, [0b0000_0010]);
		let unpacked = ClientId::unpack_smart(&packed).unwrap();
		assert_eq!(unpacked, client);

		let packed: [u8; 1] = server.pack().unwrap();
		assert_eq!(packed, [0b0000_0000]);
		let unpacked = ClientId::unpack_smart(&packed).unwrap();
		assert_eq!(unpacked, server);
	}

	#[test]
	#[should_panic]
	fn clientid_assert() {
		let _ = ClientId::new(5, Role::Server);
	}
}
