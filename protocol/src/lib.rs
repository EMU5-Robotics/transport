#![no_std]

pub use packet::{ToBrain, ToRobot};
pub use postcard;
pub use serde::{Deserialize, Serialize};

extern crate alloc;

pub mod packet {
    use crate::DevicesList;

    use serde::{Deserialize, Serialize};
    #[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
    pub enum ToBrain {
        EstablishConnection,
        Control,
        Ping(alloc::vec::Vec<u8>),
    }

    #[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
    pub enum ToRobot {
        DevicesList(DevicesList),
        Status,
        Pong(alloc::vec::Vec<u8>),
    }
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct DevicesList {
    pub smart_ports: [PortState; 20],
    pub adi_ports: [AdiPortState; 8],
}

impl DevicesList {
    pub fn motor_count(&self) -> usize {
        self.smart_ports
            .iter()
            .filter(|&v| *v == PortState::Motor)
            .count()
    }
}

#[repr(u8)]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum PortState {
    Motor,
    Encoder,
    Other(u8),
    Unplugged,
}
#[repr(u8)]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum AdiPortState {
    Undefined,
    DigitalIn,
    DigitalOut,
    AnalogIn,
    PwmOut,
    Other,
    Unplugged,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Triports(u8);

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Status {
    pub state: CompetitionState,
    pub controller_buttons: CompetitionState,
    pub selected_auton_program: u8,
}

#[allow(dead_code)]
pub struct ControllerButtons(u16);

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct CompetitionState(u8);
