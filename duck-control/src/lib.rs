//! The robot control core: everything between reading the bus and writing it.
//!
//! Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
//! owns all of that. The boundary is enforced by the compiler rather than by discipline,
//! which is what stops process concerns leaking into the code that drives motors.
//!
//! The control path it holds — model, bus, [`io::RobotIo`], observations, policy, safety — is
//! designed in `docs/design/robotd-design.md` §2.

pub mod bus;
pub mod fall;
pub mod imu;
/// Joints from the servo bus, orientation from a module that is not on it.
pub mod imu_external;
/// The bus that module sits on. Linux-only in its body; the register map is not.
pub mod imu_i2c;
/// The external IMU module, read over I²C instead of ridden along on the servo bus.
pub mod imu_icm42670p;
pub mod io;
pub mod model;
pub mod obs;
pub mod pickup;
pub mod policy;
pub mod safety;
/// A robot in MuJoCo, over TCP — the backend `robotd-design.md` §9 deferred.
pub mod sim;

pub use imu::ImuData;
pub use io::{FakeIo, IoError, JointTargets, RobotIo, Sensors, SlowSensors};
pub use model::{
    BATTERY_EMPTY_V, BATTERY_FULL_V, DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, NUM_JOINTS,
    REST_POSITION, battery_percent,
};
pub use obs::{ACTION_LEN, Command, OBS_LEN, Observation};
