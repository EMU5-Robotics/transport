use std::{
	io::Result as IoResult,
	net::{IpAddr, Ipv4Addr, TcpStream, UdpSocket},
	thread::{self, JoinHandle},
	time::{Duration, Instant},
};

use crossbeam_channel::{unbounded, Receiver, Sender};
use tungstenite::{protocol::Message as WsMessage, util::NonBlockingResult};

use common::{protocol::*, BROADCAST_MSG, BROADCAST_PORT, SERVER_PORT};

pub fn listen_for_server() -> IoResult<Ipv4Addr> {
	let socket = UdpSocket::bind(("0.0.0.0", BROADCAST_PORT))?;

	log::info!("Listening for server broadcast...");

	loop {
		let mut buf = [0; BROADCAST_MSG.len()];
		let (bytes, addr) = socket.recv_from(&mut buf)?;

		// Check the content matches
		if buf != BROADCAST_MSG {
			log::debug!("got invalid packet of {bytes} containing: {:?}", buf);
			continue;
		}

		// Check was IPv4
		let ip = match addr.ip() {
			IpAddr::V4(ip) => ip,
			_ => continue,
		};

		return Ok(ip);
	}
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClientConfiguration {
	pub prefered_name: Option<String>,
	pub role: Role,
}

impl Default for ClientConfiguration {
	fn default() -> Self {
		Self {
			prefered_name: None,
			role: if cfg!(feature = "coprocessor") {
				Role::Robot
			} else {
				Role::Client
			},
		}
	}
}

pub struct Client {
	thrd_handle: JoinHandle<()>,
	pub rx: Receiver<Packet>,
	pub tx: Sender<Packet>,
}

// TODO: packet builder
impl Client {
	/// Create a new connection to the robot server. I/O is handled by a separate
	/// thread. Use the send and receive methods to send and read packets from
	/// the server.
	pub fn connect_threaded(server: Ipv4Addr, config: ClientConfiguration) -> Self {
		let (thrd_tx, clnt_rx) = unbounded::<Packet>();
		let (clnt_tx, thrd_rx) = unbounded::<Packet>();

		let handle = std::thread::spawn(move || {
			// Connect to the websocket
			let (mut socket, resp) = do_ws_handshake((server, SERVER_PORT));
			log::info!("Connected to server: ws://{}:{}/ws", server, SERVER_PORT);
			log::debug!("Connection response: {:?}", resp);

			let mut last = Instant::now();
			let mut write_buf = Vec::new();
			loop {
				// Attempt to read an incoming packet, forward onto client if success
				match socket.read().no_block() {
					Ok(Some(WsMessage::Binary(msg))) => match Packet::decode(&msg) {
						Ok(msg) => {
							log::debug!("got msg {:?}", msg);
							if thrd_tx.try_send(msg).is_err() {
								panic!("sending channel was disconnected");
							}
						}
						Err(err) => {
							log::error!(
								"Failed to parse packet ({:?}) from server containing bytes: {:?}",
								err,
								msg
							);
							continue;
						}
					},
					Ok(_) => { /* do nothing */ }
					Err(err) => panic!("failed to read packet: {:?}", err),
				}

				// Write any outgoing packets
				while let Ok(msg) = thrd_rx.try_recv() {
					// encode it
					write_buf.clear();
					Packet::encode(&msg, &mut write_buf);
					match socket
						.write(WsMessage::Binary(write_buf.clone()))
						.no_block()
					{
						Ok(_) => { /* do nothing*/ }
						Err(err) => panic!("failed to send packet: {:?}", err),
					}
				}

				// Flush the queue
				match socket.flush().no_block() {
					Ok(_) => { /* do nothing*/ }
					Err(err) => panic!("failed to flush packet queue: {:?}", err),
				}

				// Wait a bit before checking for new packets
				let now = Instant::now();
				if let Some(time) = Duration::from_millis(5).checked_sub(now.duration_since(last)) {
					thread::sleep(time);
				}
				last = now;
			}
		});

		// Try and send a handshake packet, ignore any errors, they will be revealed later
		clnt_tx
			.send(Packet {
				source: ClientId::null(),
				target: ClientId::server(),
				timestamp: 0,
				msg: ControlMessage::ClientInit {
					name: config.prefered_name,
					role: config.role,
				},
			})
			.ok();

		Self {
			thrd_handle: handle,
			rx: clnt_rx,
			tx: clnt_tx,
		}
	}

	/// This function will block until the client is finished. It will return
	/// `true` on successful exits, and false on unsuccessful exits (panic).
	pub fn join(self) -> bool {
		self.thrd_handle.join().is_ok()
	}
}

fn do_ws_handshake(
	addr: (Ipv4Addr, u16),
) -> (
	tungstenite::protocol::WebSocket<TcpStream>,
	tungstenite::handshake::client::Response,
) {
	use tungstenite::{
		client::IntoClientRequest,
		handshake::{client::ClientHandshake, HandshakeError},
	};

	// Create a new TCP stream to the server
	let socket = TcpStream::connect(addr).expect("couldn't connect to server");
	socket.set_nonblocking(true).unwrap();

	// Start the WS handshake
	let mut conn = ClientHandshake::start(
		socket,
		format!("ws://{}:{}/ws", addr.0, addr.1)
			.into_client_request()
			.unwrap(),
		None,
	)
	.unwrap();

	// Continuously try to advance the handshake by polling, I couldn't be assed
	// figuring out if there is a proper way to get access to the socket used
	// during the handshake.
	loop {
		match conn.handshake() {
			// Handshake finished
			Ok(v) => return v,
			// Didn't block, wait a bit and try again
			Err(HandshakeError::Interrupted(c)) => {
				conn = c;
				thread::sleep(Duration::from_millis(1));
			}
			Err(err) => panic!("failed to connect to server websocket: {:?}", err),
		}
	}
}
