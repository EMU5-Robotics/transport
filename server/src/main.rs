use core::time::Duration;

use actix::prelude::*;
use actix_web::{middleware::Logger, web, App, Error, HttpRequest, HttpResponse, HttpServer};
use actix_web_actors::ws;

use crate::actors::*;
use common::SERVER_PORT;

mod actors;
mod broadcaster;

fn main() {
	let rt = tokio::runtime::Runtime::new().unwrap();

	let interface = default_interface().unwrap();

	tokio::spawn({
		let interface = interface.clone();
		async move {
			let mut brdcstr = broadcaster::Broadcaster::new(interface, Duration::from_secs(5))
				.await
				.unwrap();
			loop {
				brdcstr.broadcast().await.unwrap();
				brdcstr.wait().await;
			}
		}
	});

	rt.block_on(server_main(interface));
}

pub fn default_interface() -> Option<pnet::datalink::NetworkInterface> {
	// Check that the interface is, up, not loopback, has an IPv4 address
	pnet::datalink::interfaces()
		.iter()
		.find(|e| e.is_up() && !e.is_loopback() && e.ips.iter().any(|i| i.is_ipv4()))
		.cloned()
}

pub async fn server_main(interface: pnet::datalink::NetworkInterface) {
	let ip = interface.ips.iter().find(|i| i.is_ipv4()).unwrap();
	let addr = (ip.ip(), SERVER_PORT);

	let server = DataServer::new().start();

	HttpServer::new(move || {
		App::new()
			.app_data(web::Data::new(server.clone()))
			.route("/ws", web::get().to(server_route))
			.wrap(Logger::default())
	})
	.workers(2)
	.bind(addr)
	.unwrap()
	.run()
	.await
	.unwrap()
}

async fn server_route(
	req: HttpRequest,
	stream: web::Payload,
	srv: web::Data<Addr<DataServer>>,
) -> Result<HttpResponse, Error> {
	ws::start(ClientConnection::new(srv.get_ref().clone()), &req, stream)
}
