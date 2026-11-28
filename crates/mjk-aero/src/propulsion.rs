//! Propulsion ([#19](https://github.com/MatthewK84/MJKPhysics/issues/19)): the brushless motor,
//! its ESC, and the loads it turns. The book's *Motor and ESC* page gives the equations,
//! MOT-1 to MOT-13, and the validation.

mod esc;
mod motor;
mod system;

pub use esc::{Esc, EscError, EscState};
pub use motor::{
    Braking, DcMotor, Drive, DriveInput, LagMotor, MotorError, MotorModel, OperatingPoint,
};
pub use system::{ConstantLoad, LoadTorque, MotorSpeed, MotorSystem, PropellerLoad};
