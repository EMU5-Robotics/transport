use super::*;

use std::vec;

#[test]
fn layout_counts() {
	let mut layout = Layout::default();
	layout.ports[1] = PortState::Motor;
	layout.ports[2] = PortState::Motor;
	layout.ports[3] = PortState::Motor;
	layout.ports[4] = PortState::Motor;
	layout.ports[10] = PortState::Encoder;
	layout.ports[11] = PortState::Encoder;
	layout.ports[12] = PortState::Encoder;
	layout.ports[8] = PortState::Motor;

	assert_eq!(5, layout.motor_count());
	assert_eq!(8, layout.encoder_count());
}

#[test]
fn packet1_de_serialise() {
	let pairs = [
		(
			Packet1::default(),
			&[
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
			],
		),
		(
			{
				let mut pkt = Packet1::default();
				pkt.ports[1] = PortState::Motor;
				pkt.ports[2] = PortState::Motor;
				pkt.ports[3] = PortState::Encoder;
				pkt.ports[4] = PortState::Encoder;
				pkt.triports = 0b0010_0100;
				pkt
			},
			&[
				0xFF, 0x01, 0x01, 0x02, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x24,
			],
		),
	];

	let mut buf = vec![0; 128];

	// Should succeed
	let layout = Layout::default();
	for (pkt, bytes) in pairs {
		assert_eq!(pkt.serialise(&mut buf[..]), Ok(&bytes[..]));
		assert_eq!(Packet1::deserialise(&layout, bytes), Ok(pkt));
	}

	// Should error
	let pkt = Packet1::default();
	let res = pkt.serialise(&mut buf[..20]);
	assert_eq!(res, Err(Error::BufferOverrun));

	let bytes = &[
		0xFF, 0x01, 0xFF, 0x01, 0xFF, 0xFF, 0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	];
	let res = Packet1::deserialise(&layout, bytes);
	assert_eq!(res, Err(Error::InvalidValue));

	let res = Packet1::deserialise(&layout, &bytes[0..20]);
	assert_eq!(res, Err(Error::BadPacket));
}

#[test]
fn packet2_de_serialise() {
	let pairs = [(Packet2::default(), &[0xAA])];

	let mut buf = vec![0; 128];

	// Should succeed
	let layout = Layout::default();
	for (pkt, bytes) in pairs {
		assert_eq!(pkt.serialise(&mut buf[..]), Ok(&bytes[..]));
		assert_eq!(Packet2::deserialise(&layout, bytes), Ok(pkt));
	}

	// Should error
	let pkt = Packet2::default();
	let res = pkt.serialise(&mut buf[..0]);
	assert_eq!(res, Err(Error::BufferOverrun));

	let bytes = &[0xAA, 0xBB];
	let res = Packet2::deserialise(&layout, &bytes[1..2]);
	assert_eq!(res, Err(Error::InvalidValue));

	let res = Packet2::deserialise(&layout, bytes);
	assert_eq!(res, Err(Error::BadPacket));
}

#[test]
fn packet3_de_serialise() {
	let pairs = [
		(
			Packet3::default(),
			Layout::default(),
			&[0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00][..],
		),
		(
			{
				let mut pkt = Packet3::default();
				pkt.set_encoder(3, 2_147_483_647);
				pkt.set_encoder(4, -2_147_483_648);
				pkt.set_encoder(10, 0x01_02_03_04);
				pkt.set_motor_state(3, MotorState::default());
				pkt.set_motor_state(4, MotorState::default());
				pkt
			},
			{
				let mut layout = Layout::default();
				layout.ports[3] = PortState::Motor;
				layout.ports[4] = PortState::Motor;
				layout.ports[10] = PortState::Encoder;
				layout
			},
			&[
				0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
				0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			][..],
		),
		(
			{
				let mut pkt = Packet3::default();
				pkt.state = CompetitionState::User;
				pkt.set_encoder(12, 400);
				pkt
			},
			{
				let mut layout = Layout::default();
				layout.ports[12] = PortState::Encoder;
				layout
			},
			&[
				0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x90,
			][..],
		),
	];

	let mut buf = vec![0; 128];

	// Should succeed
	for (pkt, layout, bytes) in pairs {
		assert_eq!(pkt.serialise(&mut buf[..]), Ok(&bytes[..]));
		assert_eq!(Packet3::deserialise(&layout, bytes), Ok(pkt));
	}

	// Should error
	let pkt = Packet3::default();
	let res = pkt.serialise(&mut buf[..6]);
	assert_eq!(res, Err(Error::BufferOverrun));

	let mut layout = Layout::default();
	let bytes = &[0xAA, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00];

	let res = Packet3::deserialise(&layout, bytes);
	assert_eq!(res, Err(Error::InvalidValue));

	let res = Packet3::deserialise(&layout, &bytes[..6]);
	assert_eq!(res, Err(Error::BadPacket));

	layout.ports[2] = PortState::Motor;
	let res = Packet3::deserialise(&layout, bytes);
	assert_eq!(res, Err(Error::BadPacket));
}

#[test]
fn packet3_from_layout() {
	let mut layout = Layout::default();
	layout.ports[12] = PortState::Encoder;
	let actual = Packet3::from_layout(&layout);

	let expected = {
		let mut pkt = Packet3::default();
		pkt.state = CompetitionState::Unknown;
		pkt.encoders[12] = Some(0);
		pkt
	};

	assert_eq!(actual, expected);
}

#[test]
fn packet4_de_serialise() {
	let pairs = [
		(Packet4::default(), Layout::default(), &[0x00][..]),
		(
			{
				let mut pkt = Packet4::default();
				pkt.set_motor(3, 0x01_02);
				pkt.set_motor(4, 0x03_04);
				pkt.set_motor(5, 12_000);
				pkt.set_motor(6, -12_000);
				pkt.triport_toggle = 0xAA;
				pkt
			},
			{
				let mut layout = Layout::default();
				layout.ports[3] = PortState::Motor;
				layout.ports[4] = PortState::Motor;
				layout.ports[5] = PortState::Motor;
				layout.ports[6] = PortState::Motor;
				layout.ports[0] = PortState::Encoder;
				layout
			},
			&[0x01, 0x02, 0x03, 0x04, 0x2E, 0xE0, 0xD1, 0x20, 0xAA][..],
		),
	];

	let mut buf = vec![0; 128];

	// Should succeed
	for (pkt, layout, bytes) in pairs {
		assert_eq!(pkt.serialise(&mut buf[..]), Ok(&bytes[..]));
		assert_eq!(Packet4::deserialise(&layout, bytes), Ok(pkt));
	}

	// Should error
	let mut pkt = Packet4::default();

	let res = pkt.serialise(&mut buf[..0]);
	assert_eq!(res, Err(Error::BufferOverrun));

	pkt.set_motor(0, -1);
	let res = pkt.serialise(&mut buf[..2]);
	assert_eq!(res, Err(Error::BufferOverrun));

	let mut layout = Layout::default();
	layout.ports[2] = PortState::Motor;
	layout.ports[3] = PortState::Motor;
	let bytes = &[0xDE, 0xAD, 0xBE, 0xEF, 0x01];

	let res = Packet4::deserialise(&layout, &bytes[..4]);
	assert_eq!(res, Err(Error::BadPacket));

	layout.ports[10] = PortState::Motor;
	let res = Packet4::deserialise(&layout, bytes);
	assert_eq!(res, Err(Error::BadPacket));
}
