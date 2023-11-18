use client::network::{listen_for_server, Client, ClientConfiguration};
use common::protocol::*;

use argh::FromArgs;

/// Options
#[derive(FromArgs, PartialEq, Debug)]
struct Args {
	/// asd
	#[argh(option)]
	server: Option<String>,
}

fn main() {
	common::create_logger();

	// Try and find the server and connect to it
	let addr = listen_for_server().unwrap();
	let client = Client::connect_threaded(addr, ClientConfiguration::default());

	let mut lines = std::io::stdin().lines();
	while let Some(Ok(line)) = lines.next() {
		match line.trim() {
			"get peers" => {
				client
					.tx
					.send(Packet {
						source: ClientId::null(),
						target: ClientId::server(),
						timestamp: 0,
						msg: ControlMessage::PeerListRequest,
					})
					.ok();
			}
			"upload program" => {
				client
					.tx
					.send(Packet {
						source: ClientId::null(),
						target: ClientId::new(61, Role::Robot),
						timestamp: 0,
						msg: ControlMessage::ManageMessage(ManageMessage::UploadProgram {
							name: "prog1".into(),
							data: vec![0x69, 0x69],
						}),
					})
					.ok();
			}
			"rerun start" => {
				client
					.tx
					.send(Packet {
						source: ClientId::null(),
						target: ClientId::new(61, Role::Robot),
						timestamp: 0,
						msg: ControlMessage::ManageMessage(ManageMessage::AnnounceRerunServer(
							"".parse().unwrap(),
						)),
					})
					.ok();
			}
			_ => println!("unknown command"),
		}
	}

	client.join();
}
