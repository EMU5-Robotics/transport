use std::{
	collections::HashMap,
	time::{Duration, Instant},
};

use actix::prelude::*;
use actix_web_actors::ws;
use anyhow::Result;
use rand::{self, rngs::ThreadRng, Rng};

use common::protocol::{ClientId, ControlMessage, Packet, Role};

#[derive(Message)]
#[rtype(result = "()")]
struct ActorPacket(Packet);

#[derive(Message)]
#[rtype(result = "Option<(ClientId, String)>")]
struct ClientConnect {
	addr: Recipient<ActorPacket>,
	role: Role,
	prefered_name: Option<String>,
}

#[derive(Message)]
#[rtype(result = "()")]
struct ClientDisconnect(ClientId);

#[derive(Message)]
#[rtype(result = "()")]
struct ClientInvalidMessage(ClientId, Packet);

#[derive(Message)]
#[rtype(result = "()")]
struct PeerListRequest(ClientId);

#[derive(Message)]
#[rtype(result = "()")]
struct PacketForward(Packet);

struct ClientInfo {
	name: String,
	addr: Recipient<ActorPacket>,
}

pub struct DataServer {
	clients: HashMap<ClientId, ClientInfo>,
	epoch: Instant,
	rng: ThreadRng,
}

impl DataServer {
	pub fn new() -> Self {
		DataServer {
			clients: HashMap::new(),
			rng: rand::thread_rng(),
			epoch: Instant::now(),
		}
	}

	/// generate a random client ID that isn't in use, isn't 0 and is less than
	/// the max possible client ID
	fn gen_client_id(&mut self, role: Role) -> Option<ClientId> {
		// if for whatever reason none are left then return none
		if self.clients.len() >= ClientId::MAX_ID as usize - 1 {
			return None;
		}

		loop {
			let id = self.rng.gen::<u8>() % (ClientId::MAX_ID + 1);
			if id == 0 {
				continue;
			}
			let id = ClientId::new(id, role);
			if self.clients.get(&id).is_none() {
				return Some(id);
			}
		}
	}

	fn get_timestamp(&self) -> u32 {
		self.epoch.elapsed().as_millis() as _
	}
}

impl Actor for DataServer {
	type Context = Context<Self>;

	fn start(self) -> Addr<Self> {
		// make sure the mailbox is nice and large
		let mut ctx = Context::new();
		ctx.set_mailbox_capacity(64);
		ctx.run(self)
	}
}

impl Handler<ClientConnect> for DataServer {
	type Result = Option<(ClientId, String)>;

	fn handle(&mut self, msg: ClientConnect, _: &mut Self::Context) -> Self::Result {
		// find a new client ID
		let id = self.gen_client_id(msg.role)?;

		// get a name
		let name = match msg.prefered_name {
			Some(x) => x,
			None => match id.role() {
				Role::Robot => format!("robot-{}", id.id()),
				Role::Client => format!("client-{}", id.id()),
				_ => unreachable!(),
			},
		};

		// send the client init ack
		let timestamp = self.get_timestamp();
		msg.addr.do_send(ActorPacket(Packet {
			source: ClientId::server(),
			target: id,
			timestamp,
			msg: ControlMessage::ClientInitAck {
				given_id: id,
				name: name.clone(),
				server_last_timestamp: timestamp,
			},
		}));

		self.clients.insert(
			id,
			ClientInfo {
				name: name.clone(),
				addr: msg.addr,
			},
		);

		Some((id, name))
	}
}

impl Handler<ClientDisconnect> for DataServer {
	type Result = ();

	fn handle(&mut self, msg: ClientDisconnect, _: &mut Self::Context) -> Self::Result {
		// remove the client from the list
		self.clients.remove(&msg.0);
	}
}

impl Handler<ClientInvalidMessage> for DataServer {
	type Result = ();

	fn handle(&mut self, msg: ClientInvalidMessage, _: &mut Self::Context) -> Self::Result {
		let client = &self.clients[&msg.0];

		client.addr.do_send(ActorPacket(Packet {
			source: ClientId::server(),
			target: msg.0,
			timestamp: self.get_timestamp(),
			msg: ControlMessage::InvalidMessage(Box::new(msg.1)),
		}));
	}
}

impl Handler<PeerListRequest> for DataServer {
	type Result = ();

	fn handle(&mut self, msg: PeerListRequest, _: &mut Self::Context) -> Self::Result {
		// get a list of peers
		let peers = self
			.clients
			.iter()
			.filter(|&(&id, _)| id != msg.0)
			.map(|(&id, info)| (id, info.name.clone()))
			.collect();

		let addr = &self.clients[&msg.0].addr;
		addr.do_send(ActorPacket(Packet {
			source: ClientId::server(),
			target: msg.0,
			timestamp: self.get_timestamp(),
			msg: ControlMessage::PeerList(peers),
		}));
	}
}

impl Handler<PacketForward> for DataServer {
	type Result = ();

	fn handle(&mut self, msg: PacketForward, _: &mut Self::Context) -> Self::Result {
		// Check that the target ID exists
		match self.clients.get(&msg.0.target) {
			Some(client) => {
				// Forward the packet if it does
				client.addr.do_send(ActorPacket(msg.0));
			}
			None => {
				// If it doesn't exist than notify the sender
				self.clients[&msg.0.source]
					.addr
					.do_send(ActorPacket(Packet {
						source: ClientId::server(),
						target: msg.0.source,
						timestamp: self.get_timestamp(),
						msg: ControlMessage::UnknownClientId(msg.0.target),
					}));
			}
		}
	}
}

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(5);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

enum ClientState {
	WaitingForHandshake,
	Operating,
}

pub struct ClientConnection {
	id: ClientId,
	name: String,
	state: ClientState,
	hb: Instant,
	server: Addr<DataServer>,
	encode_scratch: Vec<u8>,
}

impl ClientConnection {
	pub fn new(server_addr: Addr<DataServer>) -> Self {
		Self {
			id: ClientId::null(),
			name: String::new(),
			state: ClientState::WaitingForHandshake,
			hb: Instant::now(),
			server: server_addr,
			encode_scratch: Vec::new(),
		}
	}
}

impl ClientConnection {
	fn hb(&self, ctx: &mut ws::WebsocketContext<Self>) {
		ctx.run_interval(HEARTBEAT_INTERVAL, move |act, ctx| {
			if Instant::now().duration_since(act.hb) > CLIENT_TIMEOUT {
				ctx.stop();
				return;
			}
			ctx.ping(b"");
		});
	}
}

impl Actor for ClientConnection {
	type Context = ws::WebsocketContext<Self>;

	fn started(&mut self, ctx: &mut Self::Context) {
		// begin heartbeat
		self.hb(ctx);
		// don't register self with data server yet, wait until we get a ClientInit message first
	}

	fn stopping(&mut self, _ctx: &mut Self::Context) -> Running {
		self.server.do_send(ClientDisconnect(self.id));
		Running::Stop
	}
}

impl Handler<ActorPacket> for ClientConnection {
	type Result = ();

	fn handle(&mut self, msg: ActorPacket, ctx: &mut Self::Context) {
		self.encode_scratch.clear();
		Packet::encode(&msg.0, &mut self.encode_scratch);
		ctx.binary(self.encode_scratch.clone());
	}
}

impl ClientConnection {
	fn handle_packet_handshake(
		&mut self,
		packet: Packet,
		ctx: &mut <ClientConnection as Actor>::Context,
	) {
		match packet.msg {
			// handle the incoming packet for the handshake
			ControlMessage::ClientInit { name, role } => {
				// send a message to connect our web socket to the data server
				self.server
					.send(ClientConnect {
						addr: ctx.address().recipient(),
						role,
						prefered_name: name,
					})
					.into_actor(self)
					.then(|res, act, ctx| {
						match res {
							Ok(Some(res)) => {
								act.id = res.0;
								act.name = res.1;
								act.state = ClientState::Operating;
							}
							_ => ctx.stop(),
						}
						fut::ready(())
					})
					.wait(ctx);
			}
			// if we get any weird packets then just close the connection
			_ => {
				ctx.stop();
			}
		}
	}

	fn handle_packet(&mut self, packet: Packet, ctx: &mut <ClientConnection as Actor>::Context) {
		match packet.msg {
			ControlMessage::PeerListRequest => {
				self.server.do_send(PeerListRequest(self.id));
			}
			ControlMessage::PeerList(_) => {
				self.server.do_send(ClientInvalidMessage(self.id, packet));
			}
			ControlMessage::ManageMessage(_) => self.handle_packet_forward(packet, ctx),
			ControlMessage::InvalidMessage(_) => {
				// if we got an invalid message packet for some reason then just close the connection
				ctx.stop();
			}
			_ => {}
		}
	}

	fn handle_packet_forward(
		&mut self,
		packet: Packet,
		_ctx: &mut <ClientConnection as Actor>::Context,
	) {
		self.server.do_send(PacketForward(Packet {
			source: self.id,
			target: packet.target,
			timestamp: packet.timestamp,
			msg: packet.msg,
		}));
	}
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for ClientConnection {
	fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
		let msg = match msg {
			Ok(msg) => msg,
			Err(_) => {
				ctx.stop();
				return;
			}
		};

		match msg {
			ws::Message::Binary(bytes) => {
				let packet = match Packet::decode(&bytes) {
					Ok(pkt) => pkt,
					Err(err) => {
						log::error!("Error decoding packet: {:?}", err);
						ctx.stop();
						return;
					}
				};

				match self.state {
					ClientState::WaitingForHandshake => self.handle_packet_handshake(packet, ctx),
					ClientState::Operating => self.handle_packet(packet, ctx),
				}
			}
			ws::Message::Ping(msg) => {
				self.hb = Instant::now();
				ctx.pong(&msg);
			}
			ws::Message::Pong(_) => {
				self.hb = Instant::now();
			}
			ws::Message::Close(reason) => {
				ctx.close(reason);
				ctx.stop();
			}
			ws::Message::Continuation(_) => {
				ctx.stop();
			}
			_ => (),
		}
	}
}
