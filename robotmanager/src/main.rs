use common::protocol::*;

use config::Config;

const HELP: &'static str = r#"
quit:          close program
help:          show this message
get-peers:     get a list other connected clients
set-target:    set a target ID
get-target:    get current target ID
recv:          list all unread packets
connect-rerun: send a Rerun connect command to the current target"#;

fn main() -> anyhow::Result<()> {
	util::create_logger();

	let mut config = Config::load_or_create()?;
	let (addr, conn) = util::quick_connect(config.last_server_addr)?;
	config.last_server_addr = Some(addr);

	let mut target_id = ClientId::null();

	let mut editor = config.create_editor()?;
	while let Ok(line) = editor.readline(">> ") {
		let mut parts = line
			.trim()
			.split(" ")
			.map(str::trim)
			.filter(|s| !s.is_empty());
		match parts.next().unwrap_or("") {
			"quit" => {
				// TODO: add clean disconnect mechanism for client
				break;
			}
			"get-peers" => {
				conn.tx
					.send(Packet {
						source: conn.id,
						target: ClientId::server(),
						timestamp: 0,
						msg: ControlMessage::PeerListRequest,
					})
					.ok();
				let pkt = conn.rx.recv()?;
				match pkt.msg {
					ControlMessage::PeerList(list) => {
						for item in &list {
							println!("id: {:>4}, name: {}", item.0.to_string(), item.1);
						}
					}
					_ => log::error!("Unexpected message returned: {:?}", pkt),
				}
			}
			"set-target" => match parts.next().unwrap_or("").parse::<ClientId>() {
				Ok(id) => target_id = id,
				Err(err) => eprintln!("unable to parse client id: {:?}", err),
			},
			"get-target" => println!("{}", target_id.to_string()),
			"connect-rerun" => {
				let ip = util::local_ip().expect("Failed to get local IP");
				conn.tx
					.send(Packet {
						source: conn.id,
						target: target_id,
						timestamp: 0,
						msg: ControlMessage::ManageMessage(ManageMessage::AnnounceRerunServer(
							(ip, 9876).into(),
						)),
					})
					.ok();
			}
			"recv" => {
				while let Ok(pkt) = conn.rx.try_recv() {
					eprintln!("{:?}\n", pkt);
				}
			}
			"help" => {
				eprintln!("{}", HELP.trim_start());
			}
			"" => {}
			_ => eprintln!("unknown command, use help to list commands"),
		}
	}

	config.save()?;
	editor.save_history(&Config::history_path()?)?;
	Ok(())
}

mod util {
	use anyhow::{Context, Result};
	use client::network::{listen_for_server, Client, ClientConfiguration};

	pub fn create_logger() {
		simple_logger::init_with_level(log::Level::Warn).unwrap();
	}

	pub fn local_ip() -> Option<std::net::IpAddr> {
		Some(
			pnet::datalink::interfaces()
				.iter()
				.find(|e| e.is_up() && !e.is_loopback() && e.ips.iter().any(|i| i.is_ipv4()))
				.cloned()?
				.ips
				.iter()
				.find(|i| i.is_ipv4())?
				.ip(),
		)
	}

	pub fn config_dir() -> Result<std::path::PathBuf> {
		let mut path = homedir::get_my_home()?.context("homedir missing")?;
		path.push(".config/robotmgr/");
		std::fs::create_dir_all(&path)?;
		Ok(path)
	}

	pub fn quick_connect(addr: Option<std::net::Ipv4Addr>) -> Result<(std::net::Ipv4Addr, Client)> {
		// First try and open the TCP socket, this is kind of stupid, but quicker than whatever the
		// default timeout is for TCP sockets.
		let addr_valid = if let Some(addr) = addr {
			let addr: std::net::SocketAddr = (addr, common::SERVER_PORT).into();
			std::net::TcpStream::connect_timeout(&addr, std::time::Duration::from_millis(100))
				.is_ok()
		} else {
			false
		};
		let addr = if addr_valid {
			addr.unwrap()
		} else {
			listen_for_server().context("Could not find server address")?
		};

		Ok((
			addr,
			Client::connect_threaded(
				addr,
				ClientConfiguration {
					prefered_name: Some("robotmgr".to_owned()),
					..ClientConfiguration::default()
				},
			),
		))
	}
}

mod config {
	use anyhow::{Context, Result};
	use std::path::PathBuf;

	#[derive(serde::Serialize, serde::Deserialize, Default)]
	pub struct Config {
		pub last_server_addr: Option<std::net::Ipv4Addr>,
	}

	pub type Editor = rustyline::Editor<(), rustyline::history::FileHistory>;

	impl Config {
		pub fn load_or_create() -> Result<Self> {
			use std::{fs::File, io::Read};

			let path = Self::config_path()?;
			match File::open(path) {
				Ok(mut file) => {
					let mut content = String::new();
					file.read_to_string(&mut content)
						.context("Failed to read config file")?;
					let config = toml::from_str(&content).context("Failed to parse config file")?;
					Ok(config)
				}
				Err(_) => {
					let config = Config::default();
					config.save()?;
					Ok(config)
				}
			}
		}

		pub fn save(&self) -> Result<()> {
			use std::{fs::OpenOptions, io::Write};

			let path = Self::config_path()?;
			let mut file = OpenOptions::new()
				.write(true)
				.create(true)
				.open(path)
				.context("Failed to create/open config file")?;
			let content = toml::to_string_pretty(self)?;
			file.write_all(content.as_bytes())?;
			Ok(())
		}

		pub fn create_editor(&self) -> Result<Editor> {
			use rustyline::{
				config::Config,
				history::{FileHistory, History},
			};

			let config = Config::builder()
				.max_history_size(1000)?
				.history_ignore_dups(true)?
				.history_ignore_space(true)
				.auto_add_history(true)
				.build();

			let history_path = Self::history_path()?;
			let mut history = FileHistory::with_config(config);
			if let Err(_) = history.load(&history_path) {
				// History file did not exist, create a new one and use that
				history
					.save(&history_path)
					.context("Failed to read/write history file")?;
			}

			Ok(Editor::with_history(config, history)?)
		}

		pub fn config_path() -> Result<PathBuf> {
			let mut path = crate::util::config_dir()?;
			path.push("config.toml");
			Ok(path)
		}

		pub fn history_path() -> Result<PathBuf> {
			let mut path = crate::util::config_dir()?;
			path.push("hist");
			Ok(path)
		}
	}
}
