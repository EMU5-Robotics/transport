use std::{
	collections::HashMap,
	time::{Duration, Instant},
};

use actix::prelude::*;
use actix_web_actors::ws;
use anyhow::Result;
use rand::{self, rngs::ThreadRng, Rng};

use common::{ClientId, Message, MessageContent, Role, MAX_CLIENT_ID};

#[derive(Message)]
#[rtype(result = "()")]
pub struct ActorMessage(pub Message);

#[derive(Message)]
#[rtype(result = "Option<(ClientId, String)>")]
pub struct Connect {
	pub addr: Recipient<ActorMessage>,
	pub role: Role,
	pub prefered_name: Option<String>,
}

struct ClientInfo {
	name: String,
	role: Role,
	addr: Recipient<ActorMessage>,
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
	fn gen_client_id(&mut self) -> Option<ClientId> {
		// if for whatever reason none are left then return none
		if self.clients.len() - 1 >= MAX_CLIENT_ID as _ {
			return None;
		}

		loop {
			let id = self.rng.gen::<ClientId>() % MAX_CLIENT_ID;
			if id == 0 {
				continue;
			}
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

impl Handler<Connect> for DataServer {
	type Result = Option<(ClientId, String)>;

	fn handle(&mut self, msg: Connect, _: &mut Self::Context) -> Self::Result {
		// find a new client ID
		let id = self.gen_client_id()?;
		let role = msg.role;

		// get a name
		let name = match msg.prefered_name {
			Some(x) => x,
			None => format!("client-{}", id),
		};

		// send the client init ack
		let timestamp = self.get_timestamp();
		msg.addr.do_send(ActorMessage(Message {
			id: 0,
			role: Role::Server,
			timestamp,
			msg: MessageContent::ClientInitAck {
				given_id: id,
				name: name.clone(),
				server_last_timestamp: timestamp,
			},
		}));

		self.clients.insert(
			id,
			ClientInfo {
				name: name.clone(),
				role,
				addr: msg.addr,
			},
		);

		Some((id, name))
	}
}

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(5);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

pub struct ClientConnection {
	id: ClientId,
	name: String,
	hb: Instant,
	addr: Addr<DataServer>,
	encode_scratch: Vec<u8>,
}

impl ClientConnection {
	pub fn new(server_addr: Addr<DataServer>) -> Self {
		Self {
			id: 0,
			name: String::new(),
			hb: Instant::now(),
			addr: server_addr,
			encode_scratch: Vec::new(),
		}
	}
}

impl ClientConnection {
	fn hb(&self, ctx: &mut ws::WebsocketContext<Self>) {
		ctx.run_interval(HEARTBEAT_INTERVAL, |act, ctx| {
			if Instant::now().duration_since(act.hb) > CLIENT_TIMEOUT {
				// TODO: notify about timeout
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
		// don't register self with data server yet, wait until we get a
		// ClientInit message first
	}

	fn stopping(&mut self, _: &mut Self::Context) -> Running {
		// TODO: notify data server
		Running::Stop
	}
}

impl Handler<ActorMessage> for ClientConnection {
	type Result = ();

	fn handle(&mut self, msg: ActorMessage, ctx: &mut Self::Context) {
		self.encode_scratch.clear();
		Message::encode(&msg.0, &mut self.encode_scratch);
		ctx.binary(self.encode_scratch.clone());
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
			ws::Message::Binary(_bytes) => {
				// TODO
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
