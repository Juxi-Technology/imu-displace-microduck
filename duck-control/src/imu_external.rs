//! A robot whose IMU is not on the servo bus.
//!
//! [`crate::bus::DynamixelIo`] reads the body IMU as slot 0 of the same `sync_read` that reads
//! the servos — the `imu_to_dxl` board pretending to be a servo. This module is the other
//! arrangement: the servos answer as themselves, and the orientation comes from a module on
//! I²C, which [`crate::imu_i2c`] fetches and [`crate::imu_icm42670p`] decodes.
//!
//! Nothing above [`RobotIo`] can tell the two apart, which is the point of the seam: the
//! observation vector, safety, fall detection and the policy take the same `Sensors` either
//! way. What changes is only who fills `Sensors::imu`.
//!
//! ## Read order matters
//!
//! [`RobotIo::read`] asks the joints first and the module second, then overwrites the IMU.
//! The other order would let a servo read failure throw away a perfectly good orientation
//! sample, and the module is the cheaper of the two.
//!
//! ## This is synchronous, deliberately, and that is a known limit
//!
//! The tick waits about a millisecond for the module. The design note for the I²C arrangement
//! (`software/飞特适配架构.md` in the replica repository) puts the module on its own thread and
//! has the loop copy the latest frame, precisely so a stuck bus cannot stall a servo tick. That
//! is the right shape for a robot and it is **not** what this does yet: the thread belongs to
//! whoever drives the loop, and this crate has no threads. Moving it is a change to the caller,
//! not to this file — the seam above stays the same.

use crate::imu_i2c::{Icm42670pI2c, RegisterBlocks};
use crate::imu_icm42670p::Icm42670pDecoder;
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};

/// Joints from one backend, orientation from an I²C module.
pub struct ExternalImuIo<Io, B> {
    joints: Io,
    module: Icm42670pI2c<B>,
    decoder: Icm42670pDecoder,
}

impl<Io: RobotIo, B: RegisterBlocks> ExternalImuIo<Io, B> {
    /// `joints` should be a bus opened with
    /// [`crate::bus::DynamixelIo::open_without_bus_imu`]; `mount` is module→trunk, measured on
    /// the bench and *not* guessable — see the calibration tool.
    pub fn new(joints: Io, module: Icm42670pI2c<B>, mount: [f64; 4]) -> Self {
        Self {
            joints,
            module,
            decoder: Icm42670pDecoder::new(mount),
        }
    }

    /// The joints half, for the startup work that is the bus's business — the register census,
    /// the missing-servo check, adopting a replacement.
    pub fn joints_mut(&mut self) -> &mut Io {
        &mut self.joints
    }
}

impl<Io: RobotIo, B: RegisterBlocks> RobotIo for ExternalImuIo<Io, B> {
    fn read(&mut self) -> Result<Sensors> {
        let mut sensors = self.joints.read()?;
        let sample = self
            .module
            .read_sample()
            .map_err(|e| IoError::Bus(format!("external imu: {e}")))?;
        // Overwrites whatever the joint backend put here. A bus opened without its board
        // reports `ImuData::default`, and a backend that fakes one would still be wrong: the
        // module is the only measurement of where the trunk is.
        sensors.imu = self.decoder.decode(&sample);
        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.joints.write(targets)
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        self.joints.set_gain(kp)
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        self.joints.set_torque(on)
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        self.joints.reboot(id)
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.joints.slow_sensors()
    }

    /// The module's own convergence, not the bus's: there is no board to converge.
    fn imu_ready(&self) -> bool {
        self.decoder.ready()
    }

    fn measures_velocity(&self) -> bool {
        self.joints.measures_velocity()
    }

    fn measures_load(&self) -> bool {
        self.joints.measures_load()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::ImuData;
    use crate::imu_i2c::{REG_GYRO, REG_QUAT, TransportError};
    use crate::io::FakeIo;
    use crate::model::{DEFAULT_POSITION, NUM_JOINTS};

    const IDENTITY: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    /// Reports the same sample every time: nose down 90°, still.
    struct NoseDown;

    impl RegisterBlocks for NoseDown {
        fn read_register(
            &mut self,
            reg: u8,
            len: usize,
        ) -> std::result::Result<Vec<u8>, TransportError> {
            let mut out = vec![0u8; len];
            match reg {
                REG_GYRO => {}
                REG_QUAT => {
                    let q = [
                        std::f32::consts::FRAC_1_SQRT_2,
                        std::f32::consts::FRAC_1_SQRT_2,
                        0.0,
                        0.0,
                    ];
                    for (chunk, value) in out.chunks_mut(4).zip(q) {
                        chunk.copy_from_slice(&value.to_le_bytes());
                    }
                }
                other => {
                    return Err(TransportError::Bus(format!("unexpected {other:#04x}")));
                }
            }
            Ok(out)
        }
    }

    /// A module that cannot be read.
    struct Dead;

    impl RegisterBlocks for Dead {
        fn read_register(
            &mut self,
            _reg: u8,
            _len: usize,
        ) -> std::result::Result<Vec<u8>, TransportError> {
            Err(TransportError::Bus("remote I/O error".to_owned()))
        }
    }

    /// The whole point of the wrapper: the module's attitude wins, even when the joint backend
    /// has an opinion. A merge that let the bus's default survive would report a robot as
    /// upright while it fell over.
    #[test]
    fn the_module_supplies_the_orientation() {
        let mut joints = FakeIo::new();
        joints.set_imu(ImuData::default());
        let mut io = ExternalImuIo::new(joints, Icm42670pI2c::new(NoseDown), IDENTITY);
        // Three reads put real values behind the decoder's median.
        for _ in 0..3 {
            io.read().expect("the fake module answers");
        }
        let sensors = io.read().expect("the fake module answers");
        assert!(
            (sensors.imu.gravity[1] + 1.0).abs() < 1e-6,
            "gravity {:?} is upright, so the module was ignored",
            sensors.imu.gravity
        );
    }

    /// The joint half must arrive untouched — the wrapper replaces one field and nothing else.
    #[test]
    fn the_joint_half_passes_through() {
        let mut joints = FakeIo::at(DEFAULT_POSITION);
        joints.set_velocities([0.25; NUM_JOINTS]);
        let mut io = ExternalImuIo::new(joints, Icm42670pI2c::new(NoseDown), IDENTITY);
        let sensors = io.read().expect("the fake module answers");
        assert_eq!(sensors.positions, DEFAULT_POSITION);
        assert_eq!(sensors.velocities, [0.25; NUM_JOINTS]);
    }

    /// Readiness is the module's, not the bus's. A bus opened without its board is never
    /// ready, so delegating this would leave a robot that never brings itself up.
    #[test]
    fn readiness_follows_the_module() {
        let mut io = ExternalImuIo::new(FakeIo::new(), Icm42670pI2c::new(NoseDown), IDENTITY);
        assert!(!io.imu_ready());
        for _ in 0..Icm42670pDecoder::READY_SAMPLES {
            io.read().expect("the fake module answers");
        }
        assert!(io.imu_ready());
    }

    /// A module that stops answering must fail the tick rather than hand back the last
    /// attitude forever: a frozen orientation looks exactly like a still robot.
    #[test]
    fn a_dead_module_fails_the_read() {
        let mut io = ExternalImuIo::new(FakeIo::new(), Icm42670pI2c::new(Dead), IDENTITY);
        assert!(matches!(io.read(), Err(IoError::Bus(_))));
    }
}
