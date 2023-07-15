use std::{
	io::Result as IoResult,
	net::{IpAddr, Ipv4Addr, UdpSocket},
	sync::atomic::{AtomicBool, Ordering},
	sync::Arc,
	thread::JoinHandle,
};

use crossbeam_channel::{unbounded, Receiver, Sender};
use tungstenite::protocol::Message as WsMessage;

use common::*;

#[cfg(feature = "coprocessor")]
pub mod coprocessor;

pub fn listen_for_server() -> IoResult<Ipv4Addr> {
	let socket = UdpSocket::bind(("0.0.0.0", BROADCAST_PORT))?;

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

struct Client {
	thrd_handle: JoinHandle<()>,
	thrd_panicked: Panicked,
	rx: Receiver<Message>,
	tx: Sender<Message>,
}

fn spawn_client_thread(server: Ipv4Addr) -> Client {
	let panicked = Panicked::new();

	let (thrd_tx, clnt_rx) = unbounded::<Message>();
	let (clnt_tx, thrd_rx) = unbounded::<Message>();

	let handle = std::thread::spawn(move || {
		let (mut socket, resp) = tungstenite::connect(&format!("ws://{}:{}", server, SERVER_PORT))
			.expect("failed to connect to server websocket");
		// socket.set_nonblocking(true).unwrap();
		// TODO: must set nonblocking

		log::info!("Connected to server: {:?}", resp);

		let mut buf = Vec::new();

		loop {
			// Attempt to read an incoming message, forward onto client if success
			match socket.read_message() {
				Ok(WsMessage::Binary(msg)) => match Message::parse(&msg) {
					Ok(msg) => {
						if thrd_tx.try_send(msg).is_err() {
							panic!("sending channel was disconnected");
						}
					}
					Err(err) => {
						log::error!(
							"Failed to parse message ({:?}) from server containing bytes: {:?}",
							err,
							msg
						);
						continue;
					}
				},
				Ok(_) => { /* do nothing */ }
				Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => { /* do nothing */
				}
				Err(err) => panic!("failed to read message: {:?}", err),
			}

			// Write any outgoing messages
			while let Ok(msg) = thrd_rx.try_recv() {
				// encode it
				buf.clear();
				Message::encode(&msg, &mut buf);
				match socket.write_message(WsMessage::Binary(buf.clone())) {
					Ok(_) => { /* do nothing*/ }
					Err(tungstenite::Error::Io(e))
						if e.kind() == std::io::ErrorKind::WouldBlock =>
					{ /* do nothing */ }
					Err(err) => panic!("failed to send message: {:?}", err),
				}
			}

			// Flush the queue
			match socket.write_pending() {
				Ok(_) => { /* do nothing*/ }
				Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => { /* do nothing */
				}
				Err(err) => panic!("failed to flush message queue: {:?}", err),
			}
		}
	});

	Client {
		thrd_handle: handle,
		thrd_panicked: panicked,
		rx: clnt_rx,
		tx: clnt_tx,
	}
}

#[derive(Debug, Clone)]
struct Panicked(Arc<AtomicBool>);

impl Panicked {
	pub fn new() -> Self {
		Panicked(Arc::new(AtomicBool::new(false)))
	}
}

impl core::ops::Drop for Panicked {
	fn drop(&mut self) {
		if std::thread::panicking() {
			self.0.store(true, Ordering::Relaxed);
		} else {
			self.0.store(false, Ordering::Relaxed);
		}
	}
}
