//! Reading the external IMU module over I²C.
//!
//! [`crate::imu_icm42670p`] decodes what this fetches. The split is deliberate: everything
//! that can be wrong about the *module* is testable without a module, and everything that
//! needs a bus is behind one trait with a single method.
//!
//! This is a blocking transport with no thread of its own. `duck-control` is not a daemon and
//! does not own scheduling — whoever reads the module decides how often, and the module's own
//! loop belongs to the caller.
//!
//! **The bus is Linux-only.** `/dev/i2c-*` exists on no other platform, so off Linux only the
//! trait, the error and the register map are compiled — there is no bus to open and nothing
//! that pretends otherwise. That keeps `cargo test --workspace` runnable on a developer's
//! machine, which is the same rule `tof` follows for the head IMU.

use crate::imu_icm42670p::Icm42670pSample;

/// The module's I²C slave address, fixed in its firmware.
pub const ADDRESS: u8 = 0x23;

/// Gyro x/y/z, `i16` LE raw counts, ±2000 dps.
pub const REG_GYRO: u8 = 0x0A;

/// Quaternion w/x/y/z, `f32` LE, scalar-first.
pub const REG_QUAT: u8 = 0x16;

/// Fusion algorithm type: 6, 9 or 10 axes. Read it to tell this module apart from another
/// device that happens to answer at [`ADDRESS`].
pub const REG_ALGO_TYPE: u8 = 0x61;

pub const GYRO_BYTES: usize = 6;
pub const QUAT_BYTES: usize = 16;

/// What the transport needs from a bus: one block read from a register.
///
/// Narrow on purpose. A fake that returns canned bytes is the whole of what a test needs to
/// cover the read path and pin the register map — which is the part most likely to be wrong
/// and the part no amount of bench time proves, because a wrong register usually still
/// returns *something*.
pub trait RegisterBlocks {
    fn read_register(&mut self, reg: u8, len: usize) -> Result<Vec<u8>, TransportError>;
}

#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("i2c: {0}")]
    Bus(String),
    #[error("register {reg:#04x} answered with {got} byte(s), the module needs {need}")]
    ShortRead { reg: u8, got: usize, need: usize },
}

/// The module, over some bus.
pub struct Icm42670pI2c<R> {
    bus: R,
}

impl<R: RegisterBlocks> Icm42670pI2c<R> {
    pub fn new(bus: R) -> Self {
        Self { bus }
    }

    /// One sample: two block reads, no transaction between them that the caller must care about.
    ///
    /// The reads are independent, so a sample can straddle a fusion update and pair a fresh
    /// quaternion with the previous gyro. At the module's 100 Hz that is 10 ms of skew, and
    /// the alternative — re-reading until both agree — buys nothing: there is no counter to
    /// compare them against, and the streams are produced by the same loop.
    pub fn read_sample(&mut self) -> Result<Icm42670pSample, TransportError> {
        let gyro = self.bus.read_register(REG_GYRO, GYRO_BYTES)?;
        if gyro.len() < GYRO_BYTES {
            return Err(TransportError::ShortRead {
                reg: REG_GYRO,
                got: gyro.len(),
                need: GYRO_BYTES,
            });
        }
        let quat = self.bus.read_register(REG_QUAT, QUAT_BYTES)?;
        if quat.len() < QUAT_BYTES {
            return Err(TransportError::ShortRead {
                reg: REG_QUAT,
                got: quat.len(),
                need: QUAT_BYTES,
            });
        }
        // Both reads are long enough, so the decode cannot fail.
        Ok(Icm42670pSample::from_registers(&gyro, &quat).expect("both reads are long enough"))
    }

    /// Which fusion the module is running: 6, 9 or 10 axes.
    ///
    /// This is also the identity check. Anything can answer at [`ADDRESS`]; a device that
    /// answers here with one of the three documented values is this module.
    pub fn fusion_algorithm(&mut self) -> Result<u8, TransportError> {
        let bytes = self.bus.read_register(REG_ALGO_TYPE, 1)?;
        bytes.first().copied().ok_or(TransportError::ShortRead {
            reg: REG_ALGO_TYPE,
            got: 0,
            need: 1,
        })
    }
}

#[cfg(target_os = "linux")]
mod bus {
    use std::path::{Path, PathBuf};

    use embedded_hal::i2c::I2c as _;
    use linux_embedded_hal::I2cdev;

    use super::{ADDRESS, Icm42670pI2c, RegisterBlocks, TransportError};

    /// Where to look when the caller names no bus.
    ///
    /// The first three are the robot's wiring, not a preference. `/dev/i2c-pihat` is the symlink
    /// the official HAT's overlay creates; `/dev/i2c-3` is the same 40-pin I²C3 the HAT's J5
    /// header breaks out; `/dev/i2c-4` is what J6 becomes once R18/R19 move to R16/R17.
    ///
    /// The last two are **not** the robot: they are the header buses a bench brings up on a
    /// Raspberry Pi (`/dev/i2c-1`) and a Jetson Orin (`/dev/i2c-7`). Reading a module there
    /// proves the transport, the module and the decode; it says nothing about the Radxa's I²C
    /// controller, which is the one part of this that only the real board can answer. The
    /// numbering differs per board *and* per JetPack release, so anyone who knows their bus
    /// should name it rather than trust this list.
    pub const BUS_CANDIDATES: [&str; 5] = [
        "/dev/i2c-pihat",
        "/dev/i2c-3",
        "/dev/i2c-4",
        "/dev/i2c-1",
        "/dev/i2c-7",
    ];

    impl RegisterBlocks for I2cdev {
        fn read_register(&mut self, reg: u8, len: usize) -> Result<Vec<u8>, TransportError> {
            let mut buf = vec![0u8; len];
            // The register address and the payload are one transaction: a write of the
            // address, a repeated start, then the read. Splitting them lets another master
            // interleave and is not what the module's own driver does.
            self.write_read(ADDRESS, &[reg], &mut buf).map_err(|e| {
                TransportError::Bus(format!("read {reg:#04x} at {ADDRESS:#04x}: {e:?}"))
            })?;
            Ok(buf)
        }
    }

    /// Open the module on the first bus that answers, or say what each one said.
    ///
    /// `named` is honoured literally: a path the caller asked for is never second-guessed, so
    /// a typo in a config file fails loudly instead of silently reading a different module.
    ///
    /// The probe is a read of the fusion-algorithm register rather than the gyro, because it
    /// is one byte and because a value only this module produces is the difference between
    /// "something acked" and "the module is here".
    pub fn open_first(named: Option<&Path>) -> Result<Icm42670pI2c<I2cdev>, TransportError> {
        let candidates: Vec<PathBuf> = match named {
            Some(path) => vec![path.to_path_buf()],
            None => BUS_CANDIDATES.iter().map(PathBuf::from).collect(),
        };

        let mut last = None;
        for path in &candidates {
            if !path.exists() {
                last = Some(TransportError::Bus(format!(
                    "{} does not exist",
                    path.display()
                )));
                continue;
            }
            let bus = match I2cdev::new(path) {
                Ok(bus) => bus,
                Err(e) => {
                    last = Some(TransportError::Bus(format!(
                        "open {}: {e:?}",
                        path.display()
                    )));
                    continue;
                }
            };
            let mut module = Icm42670pI2c::new(bus);
            match module.fusion_algorithm() {
                Ok(algo) => {
                    tracing::info!(
                        bus = %path.display(),
                        address = %format_args!("{ADDRESS:#04x}"),
                        algorithm = algo,
                        "the IMU module answered"
                    );
                    return Ok(module);
                }
                Err(e) => {
                    // A bus that exists but has nothing at ADDRESS is the common case — the
                    // module is on another header — and it must not read as a failure of the
                    // bus it was not on.
                    last = Some(TransportError::Bus(format!("{}: {e}", path.display())));
                }
            }
        }

        Err(last.unwrap_or_else(|| TransportError::Bus("no bus to look on".to_owned())))
    }
}

#[cfg(target_os = "linux")]
pub use bus::{BUS_CANDIDATES, open_first};

#[cfg(test)]
mod tests {
    use super::*;

    /// Records what it was asked for and answers from a script. Standing in for the bus is the
    /// only way to test the read path off a board, and it is the read path — register numbers
    /// and lengths — that a bench session cannot confirm, since a wrong register still returns
    /// bytes.
    struct Fake {
        gyro: Vec<u8>,
        quat: Vec<u8>,
        algo: Vec<u8>,
        asked: Vec<(u8, usize)>,
    }

    impl Fake {
        fn new() -> Self {
            let mut quat = Vec::new();
            for c in [1.0f32, 0.0, 0.0, 0.0] {
                quat.extend_from_slice(&c.to_le_bytes());
            }
            Self {
                gyro: [1000i16, 0, 0]
                    .iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect(),
                quat,
                algo: vec![6],
                asked: Vec::new(),
            }
        }
    }

    impl RegisterBlocks for Fake {
        fn read_register(&mut self, reg: u8, len: usize) -> Result<Vec<u8>, TransportError> {
            self.asked.push((reg, len));
            let src = match reg {
                REG_GYRO => &self.gyro,
                REG_QUAT => &self.quat,
                REG_ALGO_TYPE => &self.algo,
                other => {
                    return Err(TransportError::Bus(format!(
                        "unexpected register {other:#04x}"
                    )));
                }
            };
            Ok(src.iter().take(len).copied().collect())
        }
    }

    /// The register map is the one thing a bench cannot check: a wrong register that happens to
    /// exist returns plausible bytes and the robot walks with someone else's data.
    #[test]
    fn a_sample_reads_the_documented_registers() {
        let mut module = Icm42670pI2c::new(Fake::new());
        let sample = module.read_sample().expect("the fake answers");
        assert_eq!(
            module.bus.asked,
            [(REG_GYRO, GYRO_BYTES), (REG_QUAT, QUAT_BYTES)]
        );
        assert_eq!(sample.gyro, [1000, 0, 0]);
        assert_eq!(sample.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// A register that answers short is not a sample. Passing it up would let a truncated read
    /// become an orientation, and a truncated read of zeros is a robot reported as upright.
    #[test]
    fn a_short_read_is_an_error_not_a_sample() {
        let mut fake = Fake::new();
        fake.gyro.truncate(4);
        let mut module = Icm42670pI2c::new(fake);
        match module.read_sample() {
            Err(TransportError::ShortRead { reg, got, need }) => {
                assert_eq!((reg, got, need), (REG_GYRO, 4, GYRO_BYTES));
            }
            other => panic!("expected a short read, got {other:?}"),
        }
    }

    /// The quaternion is read second, so its truncation has to be blamed on its own register —
    /// a diagnosis pointing at the gyro would send someone to the wrong half of the protocol.
    #[test]
    fn a_short_quaternion_read_names_the_quaternion_register() {
        let mut fake = Fake::new();
        fake.quat.truncate(15);
        let mut module = Icm42670pI2c::new(fake);
        match module.read_sample() {
            Err(TransportError::ShortRead { reg, got, need }) => {
                assert_eq!((reg, got, need), (REG_QUAT, 15, QUAT_BYTES));
            }
            other => panic!("expected a short read, got {other:?}"),
        }
    }

    /// The bus failing must not be reported as the module answering: the first is a wiring
    /// problem and the second is a firmware one, and they send you to different places.
    #[test]
    fn a_bus_failure_is_not_swallowed() {
        struct Broken;
        impl RegisterBlocks for Broken {
            fn read_register(&mut self, _reg: u8, _len: usize) -> Result<Vec<u8>, TransportError> {
                Err(TransportError::Bus("remote I/O error".to_owned()))
            }
        }
        let mut module = Icm42670pI2c::new(Broken);
        assert!(matches!(module.read_sample(), Err(TransportError::Bus(_))));
    }

    /// The algorithm type is the identity check, so it has to read one byte from its own
    /// register — reading the gyro and hoping would identify nothing.
    #[test]
    fn the_identity_probe_reads_the_algorithm_register() {
        let mut module = Icm42670pI2c::new(Fake::new());
        assert_eq!(module.fusion_algorithm().expect("the fake answers"), 6);
        assert_eq!(module.bus.asked, [(REG_ALGO_TYPE, 1)]);
    }

    /// An empty answer is not a value. Treating it as one would have the probe accept a bus
    /// with nothing on it.
    #[test]
    fn an_empty_probe_answer_is_an_error() {
        let mut fake = Fake::new();
        fake.algo.clear();
        let mut module = Icm42670pI2c::new(fake);
        assert!(matches!(
            module.fusion_algorithm(),
            Err(TransportError::ShortRead {
                reg: REG_ALGO_TYPE,
                got: 0,
                ..
            })
        ));
    }
}
