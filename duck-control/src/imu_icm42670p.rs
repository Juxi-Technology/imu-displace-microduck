//! The external IMU module (TDK ICM-42670-P, as sold), decoded.
//!
//! Not to be confused with [`crate::imu`], which decodes the `imu_to_dxl` board that rides
//! the servo bus. This one is an I²C slave at `0x23` and does **not** share the bus with the
//! servos — a separate thread reads it, so the control tick never waits on this.
//!
//! The module ships its own fused attitude, which makes the host's job smaller than the bus
//! board's: the quaternion arrives whole, `w` included, as four `f32`. There is nothing to
//! derive and no fusion to run.
//!
//! Registers, from the module's protocol sheet:
//!
//! | reg  | bytes | contents |
//! |------|-------|----------|
//! | 0x0A | 6     | gyro x/y/z, `i16` LE raw counts, ±2000 dps |
//! | 0x16 | 16    | quaternion w/x/y/z, `f32` LE, scalar-first |
//!
//! Three properties of the module that a reader of this file would otherwise take for a bug:
//!
//! * **The raw gyro reads exactly 0 while the module believes it is still.** Measured on a
//!   stationary module: 999 of 999 frames zero on all three axes, against 1675–1707 distinct
//!   values once rotated by hand. That is a stillness gate, not a dead sensor, and it is
//!   harmless here because the policy's other IMU input — projected gravity — keeps
//!   updating. **Its threshold is far below anything the robot does**: measured against a
//!   quaternion-derived true rate, the gyro was live in 97% of intervals below 0.5 °/s and
//!   reported at 0.14 °/s, so slow movement does not switch it off and nothing a gait
//!   produces will.
//! * **It runs at 100 Hz out of the box.** The protocol sheet says the default is 25 Hz, but
//!   the unit measured 99.8 Hz frame arrival with 98.9 Hz of genuinely new values.
//! * **Roughly one read in a hundred returns an all-zero quaternion block.** Measured on the
//!   robot's bus, polling `0x16` at 100 Hz: 8 of 1000 reads came back as sixteen zero bytes,
//!   and one more had a norm of 0.195. The module appears to clear the block while its fusion
//!   writes it, so a read landing in that window sees no quaternion at all — the window is a
//!   fixed slice of each cycle, so reading *slower* collides less often, not more. This is what
//!   the norm guard below is for; an unguarded decoder would report a robot as upright every
//!   time it happened. The cost of guarding is one tick of stale attitude under 2% of the time,
//!   which is the right trade against an orientation that is simply wrong.
//! * **The quaternion is systematically short of unit.** Measured over the same 1000 reads:
//!   mean norm 0.9975, and never above 0.9983 — it does not round up to 1. The acceptance band
//!   is far wider than that on purpose, because its job is to catch a zeroed or mangled block,
//!   not to police precision.
//! * **The first sample after start-up is reported as the initial state, not as the
//!   measurement.** `median3_each` takes a median over the two prior samples and the current
//!   one, and both histories start at their defaults — gyro zero, gravity upright. A median
//!   of (initial, initial, measured) is the initial value, so the first tick of a robot held
//!   sideways still reads as perfectly upright. From the second sample on the median is over
//!   two real values and is exact. The bus board behaves the same way, and this is the
//!   sharpest reason nothing may act on the orientation before [`Icm42670pDecoder::ready`].
//!
//! As with the bus board, mount rotation is applied here and samples arrive in the module's
//! own axes — nothing upstream of this file pre-rotates anything.

use crate::imu::{ImuData, median3_each, mul, normalise, rotate, rotate_inverse};

/// Gyro full scale: ±2000 dps across the `i16` range. The protocol sheet rounds this to
/// 0.061 (°/s)/LSB; the exact figure is 0.061037.
const GYRO_DPS_PER_LSB: f64 = 2000.0 / 32767.0;

/// Gyro counts → rad/s. 32767 counts is exactly 2000 dps.
pub const GYRO_RAD_PER_LSB: f64 = GYRO_DPS_PER_LSB * std::f64::consts::PI / 180.0;

/// Accepted `|q|` for a sample to count as the module's fused output.
///
/// The module ships `f32`, so this is a real unit quaternion and can be held to a real unit
/// norm — unlike the bus board, whose half-precision components need headroom for rounding
/// at full scale. A fresh unit measured 0.9983. Outside the band is a short read, a bus
/// glitch or a cold start, none of which is a measurement.
const QUAT_NORM: (f64, f64) = (0.9, 1.1);

/// One read of the module, in the module's own axes — no scaling applied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Icm42670pSample {
    /// Gyro x/y/z, raw counts, ±2000 dps.
    pub gyro: [i16; 3],
    /// Quaternion w/x/y/z, scalar-first, as the module fused it.
    pub quat: [f32; 4],
}

impl Icm42670pSample {
    /// Decode the module's two register reads: `0x0A` (6 bytes) and `0x16` (16 bytes).
    ///
    /// `None` on a short slice. A truncated read is not a sample, and filling the gap with
    /// zeros would look exactly like a robot standing still and perfectly upright — the one
    /// lie fall detection must never be told.
    pub fn from_registers(gyro: &[u8], quat: &[u8]) -> Option<Self> {
        if gyro.len() < 6 || quat.len() < 16 {
            return None;
        }
        Some(Self {
            gyro: [
                i16::from_le_bytes([gyro[0], gyro[1]]),
                i16::from_le_bytes([gyro[2], gyro[3]]),
                i16::from_le_bytes([gyro[4], gyro[5]]),
            ],
            quat: [
                f32::from_le_bytes([quat[0], quat[1], quat[2], quat[3]]),
                f32::from_le_bytes([quat[4], quat[5], quat[6], quat[7]]),
                f32::from_le_bytes([quat[8], quat[9], quat[10], quat[11]]),
                f32::from_le_bytes([quat[12], quat[13], quat[14], quat[15]]),
            ],
        })
    }
}

/// Decodes the module's samples into [`ImuData`].
///
/// State and rules deliberately mirror [`crate::imu::SflpDecoder`], because the consumers
/// above cannot tell the two apart and should not have to: the last good quaternion is held
/// across a bad read rather than snapped to identity, and orientation is not called measured
/// until it has been seen [`Icm42670pDecoder::READY_SAMPLES`] times.
pub struct Icm42670pDecoder {
    /// Module→trunk mounting rotation, scalar-first. Measured on the bench, never assumed.
    mount: [f64; 4],
    /// Last quaternion the module actually produced.
    last_quat: [f64; 4],
    /// Samples carrying a usable quaternion. Gates [`Icm42670pDecoder::ready`].
    quat_samples: u32,
    gyro_history: [[f64; 3]; 2],
    gravity_history: [[f64; 3]; 2],
}

impl Icm42670pDecoder {
    /// Samples before the orientation counts as measured — 0.25 s at the module's 100 Hz.
    pub const READY_SAMPLES: u32 = 25;

    /// `mount` is module→trunk. There is no default: a guessed mounting angle is the one
    /// error that passes every electrical check and still makes the robot fall the wrong way.
    pub fn new(mount: [f64; 4]) -> Self {
        Self {
            mount,
            last_quat: [1.0, 0.0, 0.0, 0.0],
            quat_samples: 0,
            gyro_history: [[0.0; 3]; 2],
            gravity_history: [[0.0, 0.0, -1.0]; 2],
        }
    }

    /// Whether the module has demonstrably produced fused output.
    ///
    /// Until this is true the orientation is a default, not a measurement, and fall
    /// detection must not run on it.
    pub fn ready(&self) -> bool {
        self.quat_samples >= Self::READY_SAMPLES
    }

    /// Quaternions accepted so far, which is [`Self::decode`] calls minus the rejected ones.
    ///
    /// Worth watching on a new module: a rejection is silent to every consumer, because the
    /// previous orientation is held rather than reported as bad. A handful over a start-up is
    /// the module settling; a steady trickle means the policy is being fed a stale attitude
    /// every so often, and neither the loop nor fall detection can tell.
    pub fn quat_samples(&self) -> u32 {
        self.quat_samples
    }

    pub fn decode(&mut self, sample: &Icm42670pSample) -> ImuData {
        let gyro_sensor = [
            f64::from(sample.gyro[0]) * GYRO_RAD_PER_LSB,
            f64::from(sample.gyro[1]) * GYRO_RAD_PER_LSB,
            f64::from(sample.gyro[2]) * GYRO_RAD_PER_LSB,
        ];
        let gyro = rotate(self.mount, gyro_sensor);

        let sensor = [
            f64::from(sample.quat[0]),
            f64::from(sample.quat[1]),
            f64::from(sample.quat[2]),
            f64::from(sample.quat[3]),
        ];
        let norm = (sensor[0] * sensor[0]
            + sensor[1] * sensor[1]
            + sensor[2] * sensor[2]
            + sensor[3] * sensor[3])
            .sqrt();
        // `norm` is NaN for a corrupted read and every comparison against NaN is false, so
        // the range check rejects those too.
        if sensor.iter().all(|c| c.is_finite()) && (QUAT_NORM.0..=QUAT_NORM.1).contains(&norm) {
            let mount_inv = [
                self.mount[0],
                -self.mount[1],
                -self.mount[2],
                -self.mount[3],
            ];
            let q = mul(sensor, mount_inv);
            let q_norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
            if q_norm > 0.5 {
                self.last_quat = [q[0] / q_norm, q[1] / q_norm, q[2] / q_norm, q[3] / q_norm];
                self.quat_samples = self.quat_samples.saturating_add(1);
            }
        }

        let gravity = normalise(rotate_inverse(self.last_quat, [0.0, 0.0, -1.0]));

        let out = ImuData {
            gyro: median3_each(&self.gyro_history, gyro),
            gravity: median3_each(&self.gravity_history, gravity),
            quat: self.last_quat,
        };
        self.gyro_history = [self.gyro_history[1], gyro];
        self.gravity_history = [self.gravity_history[1], gravity];
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQRT_HALF: f32 = std::f32::consts::FRAC_1_SQRT_2;

    const IDENTITY: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    /// The `zero3`-style mounting the bus board uses: module axes sit +90° about Y from the
    /// trunk's. A fixture only — a real mount comes from the bench calibration.
    const MOUNT_Y90: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    /// A quaternion the module actually produced, flat on a bench and yawed ~75°, captured
    /// over the serial link. Replaying real hardware beats testing only with values we
    /// picked to be convenient.
    const MEASURED: [f32; 4] = [0.7923, 0.0039, -0.0071, -0.6073];

    fn sample(gyro: [i16; 3], quat: [f32; 4]) -> Icm42670pSample {
        Icm42670pSample { gyro, quat }
    }

    /// The median takes the two prior samples with it, so the very first output is the
    /// initial history rather than the measurement. Three decodes leave three real values
    /// behind the median, which is what every assertion below needs.
    fn decode_warm(d: &mut Icm42670pDecoder, s: &Icm42670pSample) -> ImuData {
        d.decode(s);
        d.decode(s);
        d.decode(s)
    }

    /// This is what turns counts into the rad/s the policy was trained on. Pinned against
    /// the protocol sheet's rounded 0.061 (°/s)/LSB, and against the full-scale endpoint,
    /// which must land on exactly ±2000 dps — a wrong scale is a silently wrong gait.
    #[test]
    fn gyro_scale_matches_the_protocol_sheet() {
        let dps_per_lsb = GYRO_RAD_PER_LSB * 180.0 / std::f64::consts::PI;
        assert!((dps_per_lsb - 0.061).abs() < 5e-5, "{dps_per_lsb}");
        assert!((32767.0 * dps_per_lsb - 2000.0).abs() < 1e-6);
    }

    /// No mount, no rotation: what the module says is what the trunk gets.
    #[test]
    fn identity_mount_passes_the_gyro_through() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        let out = decode_warm(&mut d, &sample([1000, 0, 0], [1.0, 0.0, 0.0, 0.0]));
        assert!(
            (out.gyro[0] - 1000.0 * GYRO_RAD_PER_LSB).abs() < 1e-9,
            "{:?}",
            out.gyro
        );
        assert_eq!(out.gyro[1], 0.0);
        assert_eq!(out.gyro[2], 0.0);
    }

    /// A +90° mount about Y sends a module +X rate to trunk −Z. Sign errors here are the
    /// ones that pass every electrical check and still make the duck fall the wrong way.
    #[test]
    fn mount_rotates_the_gyro_into_the_trunk_frame() {
        let mut d = Icm42670pDecoder::new(MOUNT_Y90);
        let out = decode_warm(&mut d, &sample([1000, 0, 0], [1.0, 0.0, 0.0, 0.0]));
        assert!(out.gyro[0].abs() < 1e-9, "{:?}", out.gyro);
        assert!(out.gyro[1].abs() < 1e-9, "{:?}", out.gyro);
        assert!(
            (out.gyro[2] + 1000.0 * GYRO_RAD_PER_LSB).abs() < 1e-9,
            "{:?}",
            out.gyro
        );
    }

    /// Projected gravity is the other half of the policy's IMU input, and the only part that
    /// says which way is down. Pinned for the attitudes the bench calibration puts the robot
    /// in, because a mount that is right about gyro and wrong about gravity still falls.
    #[test]
    fn gravity_follows_the_quaternion() {
        let cases: [(&str, [f32; 4], [f64; 3]); 4] = [
            ("level", [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            (
                "+90° about X",
                [SQRT_HALF, SQRT_HALF, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ),
            (
                "−90° about X",
                [SQRT_HALF, -SQRT_HALF, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ),
            (
                "+90° about Y",
                [SQRT_HALF, 0.0, SQRT_HALF, 0.0],
                [1.0, 0.0, 0.0],
            ),
        ];
        for (name, q, want) in cases {
            let mut d = Icm42670pDecoder::new(IDENTITY);
            let out = decode_warm(&mut d, &sample([0, 0, 0], q));
            for i in 0..3 {
                assert!(
                    (out.gravity[i] - want[i]).abs() < 1e-6,
                    "{name}: gravity {:?} != {want:?}",
                    out.gravity
                );
            }
        }
    }

    /// The decode stays honest against hardware, not just against quaternions chosen to be
    /// convenient. Expects the values a Python re-implementation of these helpers produced
    /// for this same sample.
    #[test]
    fn decodes_a_measured_sample() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        let out = decode_warm(&mut d, &sample([0, 0, 0], MEASURED));
        let want = [-0.0065, -0.0149, -0.9999];
        for i in 0..3 {
            assert!(
                (out.gravity[i] - want[i]).abs() < 1e-3,
                "gravity {:?} != {want:?}",
                out.gravity
            );
        }
        // Arrived at |q| = 0.9983; normalising is part of the decode.
        let norm = out.quat.iter().map(|c| c * c).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-9, "|q| = {norm}");
    }

    /// A read that never came from the fusion must not move the orientation. The all-zero
    /// case is what a failed I²C transaction most often leaves behind, and a decoder that
    /// accepted it would report a robot as upright while it fell.
    #[test]
    fn bad_quaternions_do_not_move_the_orientation() {
        for (name, bad) in [
            ("all zero", [0.0f32, 0.0, 0.0, 0.0]),
            ("not a number", [f32::NAN, 0.0, 0.0, 0.0]),
            ("infinite", [f32::INFINITY, 0.0, 0.0, 0.0]),
            ("twice unit", [2.0, 0.0, 0.0, 0.0]),
            ("0.6 short", [0.6, 0.0, 0.0, 0.0]),
        ] {
            let mut d = Icm42670pDecoder::new(IDENTITY);
            let before = d.decode(&sample([0, 0, 0], MEASURED)).quat;
            let after = d.decode(&sample([0, 0, 0], bad)).quat;
            assert_eq!(before, after, "{name} moved the quaternion");
            assert_eq!(d.quat_samples, 1, "{name} was counted as a sample");
        }
    }

    /// The stillness gate: the module zeroes its raw gyro while it thinks it is still, so a
    /// decoder that needed a rate to trust the attitude would lose orientation the moment
    /// the robot held still. Projected gravity is the part that keeps working.
    #[test]
    fn a_still_module_still_reports_attitude() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        let out = decode_warm(&mut d, &sample([0, 0, 0], [SQRT_HALF, SQRT_HALF, 0.0, 0.0]));
        assert_eq!(out.gyro, [0.0, 0.0, 0.0]);
        assert!((out.gravity[1] + 1.0).abs() < 1e-6, "{:?}", out.gravity);
    }

    /// A dropped read shows up as one wild value; the median drops it without an average's
    /// lag. The failure it prevents is a single dropped I²C transaction throwing the policy
    /// an angular velocity it never had.
    #[test]
    fn a_spike_between_two_good_samples_is_discarded() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        let q = [1.0f32, 0.0, 0.0, 0.0];
        d.decode(&sample([1000, 0, 0], q));
        d.decode(&sample([1000, 0, 0], q));
        let spiked = d.decode(&sample([32767, 0, 0], q));
        assert!(
            (spiked.gyro[0] - 1000.0 * GYRO_RAD_PER_LSB).abs() < 1e-9,
            "spike leaked through: {}",
            spiked.gyro[0]
        );
    }

    /// Fall detection must not run before the orientation is a measurement rather than the
    /// initial default — the two-sample median warm-up is exactly when that matters most.
    #[test]
    fn ready_waits_for_twenty_five_quaternions() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        let q = [1.0f32, 0.0, 0.0, 0.0];
        for n in 1..Icm42670pDecoder::READY_SAMPLES {
            d.decode(&sample([0, 0, 0], q));
            assert!(!d.ready(), "ready after {n} samples");
        }
        d.decode(&sample([0, 0, 0], q));
        assert!(
            d.ready(),
            "not ready after {}",
            Icm42670pDecoder::READY_SAMPLES
        );
    }

    /// A bad read must not advance readiness, or a module wired to nothing would look ready
    /// after 25 attempts and fall detection would be armed over a default orientation.
    #[test]
    fn bad_samples_do_not_count_towards_readiness() {
        let mut d = Icm42670pDecoder::new(IDENTITY);
        for _ in 0..100 {
            d.decode(&sample([0, 0, 0], [0.0, 0.0, 0.0, 0.0]));
        }
        assert!(!d.ready());
        assert_eq!(d.quat_samples, 0);
    }

    /// The module's registers are little-endian throughout; reading them big-endian flips
    /// the sign of every rate and mirrors the horizon.
    #[test]
    fn register_reads_decode_little_endian() {
        let gyro: Vec<u8> = [1000i16, -1000, 32767]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let quat: Vec<u8> = [1.0f32, 0.0, 0.0, 0.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let s = Icm42670pSample::from_registers(&gyro, &quat).expect("full reads decode");
        assert_eq!(s.gyro, [1000, -1000, 32767]);
        assert_eq!(s.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// A short read is not a sample. Reporting it as one is how a robot that lost its IMU
    /// gets reported as standing perfectly still.
    #[test]
    fn short_reads_are_not_samples() {
        assert!(Icm42670pSample::from_registers(&[0; 5], &[0; 16]).is_none());
        assert!(Icm42670pSample::from_registers(&[0; 6], &[0; 15]).is_none());
        assert!(Icm42670pSample::from_registers(&[0; 6], &[0; 16]).is_some());
    }

    /// Extra trailing bytes are the transport's business, not the decoder's — a burst read
    /// that overshoots the register of interest still carries the sample.
    #[test]
    fn longer_reads_are_accepted() {
        assert!(Icm42670pSample::from_registers(&[1, 0, 0, 0, 0, 0, 9, 9], &[0; 20]).is_some());
    }
}
