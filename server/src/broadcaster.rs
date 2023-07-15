use std::{net::IpAddr, time::Duration};

use anyhow::Result;
use pnet::datalink::NetworkInterface;
use tokio::{net::UdpSocket, time::Interval};

use common::{BROADCAST_MSG, BROADCAST_PORT};

pub struct Broadcaster {
	socket: UdpSocket,
	interval: Interval,
	boardcast_ip: IpAddr,
}

impl Broadcaster {
	pub async fn new(interface: NetworkInterface, period: Duration) -> Result<Broadcaster> {
		let ip = interface.ips.iter().find(|i| i.is_ipv4()).unwrap();

		// Bind and connect to the broadcast address
		let socket = UdpSocket::bind((ip.ip(), 0)).await?;
		socket.set_broadcast(true)?;

		let mut interval = tokio::time::interval(period);
		interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

		Ok(Broadcaster {
			socket,
			interval,
			boardcast_ip: ip.broadcast(),
		})
	}

	/// Send a single broadcast packet out to the network
	pub async fn broadcast(&self) -> Result<()> {
		self.socket
			.send_to(BROADCAST_MSG, (self.boardcast_ip, BROADCAST_PORT))
			.await?;
		Ok(())
	}

	/// Wait until the next period occurs, this can be used in another thread as a simple way of
	/// continuously broadcasting the address.
	pub async fn wait(&mut self) {
		self.interval.tick().await;
	}
}
