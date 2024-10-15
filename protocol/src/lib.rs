#![no_std]

pub use packet::{ToBrain, ToRobot};
pub use postcard;
pub use serde::{Deserialize, Serialize};

extern crate alloc;

pub mod packet {
    use crate::DevicesList;

    use serde::{Deserialize, Serialize};
    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    #[allow(clippy::large_enum_variant)]
    pub enum ToBrain {
        RequestDeviceInfo,
        RequestControllerInfo,
        RequestCompState,
        RequestEncoderState,
        SetMotors([crate::MotorControl; 21]),
        SetMotorGearsets([crate::GearSetChange; 21]),
        Ping(alloc::vec::Vec<u8>),
    }

    #[derive(Serialize, Deserialize, Debug, PartialEq)]
    #[allow(clippy::large_enum_variant)]
    pub enum ToRobot {
        DevicesList(DevicesList),
        ControllerState(crate::ControllerState),
        EncoderState([crate::EncoderState; 21]),
        CompState(crate::CompState),
        Pong(alloc::vec::Vec<u8>),
    }
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum CompState {
    Driver,
    Auton,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct DevicesList {
    pub smart_ports: [PortState; 21],
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
    Other,
    Unplugged,
}
#[repr(u8)]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum AdiPortState {
    Other,
    Unplugged,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Triports(u8);

#[derive(Serialize, Deserialize, Debug, PartialEq, Default, Clone, Copy)]
pub enum EncoderState {
    #[default]
    None,
    Radians(f64),
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Status {
    pub state: CompetitionState,
    pub controller_buttons: CompetitionState,
    pub selected_auton_program: u8,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct ControllerState {
    pub buttons: ControllerButtons,
    pub axis: [f64; 4],
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct ControllerButtons(u16);
pub mod controller {
    use crate::ControllerButtons;
    pub const NONE: ControllerButtons = ControllerButtons(0);
    pub const A: ControllerButtons = ControllerButtons(1);
    pub const B: ControllerButtons = ControllerButtons(1 << 1);
    pub const X: ControllerButtons = ControllerButtons(1 << 2);
    pub const Y: ControllerButtons = ControllerButtons(1 << 3);
    pub const UP: ControllerButtons = ControllerButtons(1 << 4);
    pub const DOWN: ControllerButtons = ControllerButtons(1 << 5);
    pub const LEFT: ControllerButtons = ControllerButtons(1 << 6);
    pub const RIGHT: ControllerButtons = ControllerButtons(1 << 7);
    pub const LEFT_TRIGGER_1: ControllerButtons = ControllerButtons(1 << 8);
    pub const LEFT_TRIGGER_2: ControllerButtons = ControllerButtons(1 << 9);
    pub const RIGHT_TRIGGER_1: ControllerButtons = ControllerButtons(1 << 10);
    pub const RIGHT_TRIGGER_2: ControllerButtons = ControllerButtons(1 << 11);
}

impl core::ops::BitOr for ControllerButtons {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}
impl core::ops::BitAnd for ControllerButtons {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}
impl core::ops::BitXor for ControllerButtons {
    type Output = Self;
    fn bitxor(self, rhs: Self) -> Self::Output {
        Self(self.0 ^ rhs.0)
    }
}
impl core::ops::BitOrAssign for ControllerButtons {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}
impl core::ops::BitAndAssign for ControllerButtons {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}
impl core::ops::BitXorAssign for ControllerButtons {
    fn bitxor_assign(&mut self, rhs: Self) {
        self.0 ^= rhs.0;
    }
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct CompetitionState(u8);

#[derive(Serialize, Deserialize, Debug, PartialEq, Default, Clone, Copy)]
pub enum MotorControl {
    #[default]
    None,
    BrakeCoast,
    BrakeBrake,
    BrakeHold,
    Voltage(f64),
    Velocity(i32),
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default, Clone, Copy)]
pub enum GearSetChange {
    #[default]
    None,
    Red,
    Green,
    Blue,
}

#[cfg(feature = "vexide")]
use vexide::devices::smart::{
    motor::{self, BrakeMode},
    SmartDeviceType,
};

#[cfg(feature = "vexide")]
impl Into<Option<motor::MotorControl>> for MotorControl {
    fn into(self) -> Option<motor::MotorControl> {
        Some(match self {
            Self::None => return None,
            Self::BrakeCoast => motor::MotorControl::Brake(BrakeMode::Coast),
            Self::BrakeBrake => motor::MotorControl::Brake(BrakeMode::Brake),
            Self::BrakeHold => motor::MotorControl::Brake(BrakeMode::Hold),
            Self::Voltage(v) => motor::MotorControl::Voltage(v),
            Self::Velocity(v) => motor::MotorControl::Velocity(v),
        })
    }
}

#[cfg(feature = "vexide")]
impl Into<Option<motor::Gearset>> for GearSetChange {
    fn into(self) -> Option<motor::Gearset> {
        Some(match self {
            Self::None => return None,
            Self::Red => motor::Gearset::Red,
            Self::Green => motor::Gearset::Green,
            Self::Blue => motor::Gearset::Blue,
        })
    }
}

#[cfg(feature = "vexide")]
impl From<SmartDeviceType> for PortState {
    fn from(v: SmartDeviceType) -> Self {
        match v {
            SmartDeviceType::None => Self::Unplugged,
            SmartDeviceType::Motor => Self::Motor,
            SmartDeviceType::Rotation => Self::Encoder,
            _ => Self::Other,
        }
    }
}
