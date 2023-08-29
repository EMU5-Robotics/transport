use fern::colors::{Color, ColoredLevelConfig};

pub mod protocol;

pub const BROADCAST_PORT: u16 = 17746;
pub const SERVER_PORT: u16 = 17747;
pub const BROADCAST_MSG: &[u8] = b"VEX RS BROADCAST SERVER";

pub fn create_logger() {
	let colors = ColoredLevelConfig::new()
		.error(Color::Red)
		.warn(Color::Yellow)
		.info(Color::Cyan)
		.debug(Color::Magenta);

	fern::Dispatch::new()
		.format(move |out, message, record| {
			out.finish(format_args!(
				"{} {} [{}] {}",
				chrono::Local::now().format("%H:%M:%S"),
				colors.color(record.level()),
				record.target(),
				message
			))
		})
		.level(log::LevelFilter::Debug)
		.chain(std::io::stderr())
		.apply()
		.unwrap();
}
