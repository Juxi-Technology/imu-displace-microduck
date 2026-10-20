//! Drive the real IMU through the [`RobotIo`] seam and out the other side as the policy's
//! observation.
//!
//! This is the third step's instrument. The transport is already proven on its own by
//! `imu_probe`; what that cannot say is whether the six numbers the policy takes from the IMU
//! arrive in the right slots. `obs.rs` calls itself "the highest-risk code in the crate"
//! because a wrong index does not fail loudly — it produces a plausible robot that falls over.
//! So this reads the real module every tick, builds the real observation through the real
//! entry point, and checks the two things that would be silently wrong:
//!
//! * **`obs[0..3]` is the gyro and `obs[3..6]` is the projected gravity**, compared against
//!   the `ImuData` that went in. Anything else and the policy is being lied to.
//! * **the joint blocks are zero**, because the fake joints report the home pose and the
//!   policy sees positions *relative* to it. A non-zero here means the home pose and the
//!   backend disagree, which is a constant offset on fourteen slots.
//!
//! ```text
//! cargo run -p duck-control --example imu_to_observation             # find the module, 20 s
//! cargo run -p duck-control --example imu_to_observation -- /dev/i2c-7 60
//! ```
//!
//! ## What this does not prove
//!
//! The joints are fake, so nothing here says anything about the servo bus, and the mount is
//! uncalibrated, so the attitude is in the module's own axes rather than the trunk's. Both are
//! stated in the output rather than left to be discovered. What it does prove is that a real
//! IMU survives the whole path from `/dev/i2c-*` to the exact slots the policy reads — which
//! leaves only the bus half for the day the servos arrive.
//!
//! ## What the bench backend is not
//!
//! `slow_sensors` reports zeros. There are no servos to ask, and the trait has no way to say
//! "not measured" — only `measures_velocity` and `measures_load` carry that distinction, and
//! a supply voltage has no equivalent. A consumer watching the battery would read 0 V, which
//! is exactly what this is: a bench, not a robot.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The IMU module is reached through /dev/i2c-*, which exists only on Linux.");
    eprintln!("Run this on the robot, or on any Linux board with the module attached.");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() {
    bench::run();
}

#[cfg(target_os = "linux")]
mod bench {
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use duck_control::imu_i2c::{Icm42670pI2c, RegisterBlocks, open_first};
    use duck_control::imu_icm42670p::Icm42670pDecoder;
    use duck_control::io::{IoError, JointTargets, RobotIo, Sensors, SlowSensors};
    use duck_control::{ACTION_LEN, Command, DEFAULT_POSITION, NUM_JOINTS, Observation};

    /// The control loop's rate, so the tick budget this exercises is the robot's.
    const RATE_HZ: u64 = 50;
    const PERIOD: Duration = Duration::from_micros(1_000_000 / RATE_HZ);

    /// Print this often. At 50 Hz a line per tick buries the reading that matters.
    const PRINT_EVERY: u64 = 25;

    /// Without a mount there is no rotation to apply, and inventing one would make the output
    /// agree with a robot that does not exist.
    const UNROTATED: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    pub fn run() {
        let mut bus: Option<PathBuf> = None;
        let mut seconds = 20.0f64;
        for arg in std::env::args().skip(1) {
            match arg.parse::<f64>() {
                Ok(secs) => seconds = secs,
                Err(_) => bus = Some(PathBuf::from(arg)),
            }
        }

        let module = match open_first(bus.as_deref()) {
            Ok(module) => module,
            Err(e) => {
                eprintln!("no module: {e}");
                eprintln!(
                    "checked: {}",
                    duck_control::imu_i2c::BUS_CANDIDATES.join(", ")
                );
                eprintln!("`i2cdetect -l` lists the buses; pass the one the module is on.");
                std::process::exit(1);
            }
        };

        println!(
            "{} Hz for {seconds:.0} s — fake joints at the home pose, real IMU, uncalibrated mount",
            RATE_HZ
        );
        println!("attitude below is in the module's own axes until the mount is measured.");
        println!();

        let io = ImuOnlyIo::new(module, UNROTATED);
        let report = drive(io, Duration::from_secs_f64(seconds));
        report.print();
        if !report.passed() {
            std::process::exit(1);
        }
    }

    /// Servos that hold the home pose and a real IMU.
    ///
    /// Generic over the register transport rather than over `/dev/i2c-*`, so the joint half and
    /// the wiring to [`Sensors`] can be exercised with a fake bus — the same split that lets
    /// the decode be tested without a module.
    struct ImuOnlyIo<B: RegisterBlocks> {
        module: Icm42670pI2c<B>,
        decoder: Icm42670pDecoder,
        /// Where the fake joints are held. The observation subtracts the home pose from this,
        /// so holding it *at* home is what makes the joint blocks zero.
        pose: [f64; NUM_JOINTS],
    }

    impl<B: RegisterBlocks> ImuOnlyIo<B> {
        fn new(module: Icm42670pI2c<B>, mount: [f64; 4]) -> Self {
            Self {
                module,
                decoder: Icm42670pDecoder::new(mount),
                pose: DEFAULT_POSITION,
            }
        }
    }

    impl<B: RegisterBlocks> RobotIo for ImuOnlyIo<B> {
        fn read(&mut self) -> duck_control::io::Result<Sensors> {
            let sample = self
                .module
                .read_sample()
                .map_err(|e| IoError::Bus(format!("imu: {e}")))?;
            Ok(Sensors {
                positions: self.pose,
                velocities: [0.0; NUM_JOINTS],
                currents_ma: [0.0; NUM_JOINTS],
                imu: self.decoder.decode(&sample),
            })
        }

        // A bench has nothing to drive. Each of these says so rather than pretending.
        fn write(&mut self, _targets: &JointTargets) -> duck_control::io::Result<()> {
            Ok(())
        }
        fn set_gain(&mut self, _kp: u16) -> duck_control::io::Result<()> {
            Ok(())
        }
        fn set_torque(&mut self, _on: bool) -> duck_control::io::Result<()> {
            Ok(())
        }
        fn reboot(&mut self, _id: u8) -> duck_control::io::Result<()> {
            Ok(())
        }
        fn slow_sensors(&mut self) -> duck_control::io::Result<SlowSensors> {
            Ok(SlowSensors {
                volts: 0.0,
                temps_c: [0.0; NUM_JOINTS],
            })
        }

        // The two declarations that stop zeros being read as measurements: these joints are
        // not moving and nothing is measuring their current, and saying so is the difference
        // between "still" and "unmeasured".
        fn measures_velocity(&self) -> bool {
            false
        }
        fn measures_load(&self) -> bool {
            false
        }

        fn imu_ready(&self) -> bool {
            self.decoder.ready()
        }
    }

    struct Report {
        seconds: f64,
        ticks: u64,
        read_failures: u64,
        stale_ticks: u64,
        bad_gyro: u64,
        bad_gravity: u64,
        bad_joints: u64,
        ready_after: Option<u64>,
        last_gyro: [f64; 3],
        last_gravity: [f64; 3],
        last_norm: f64,
    }

    impl Report {
        fn passed(&self) -> bool {
            // Nothing is allowed to be wrong that the robot would not notice. A rejection is
            // not a failure — the decoder is supposed to hold through one — but a wrong slot
            // or a non-zero joint block is exactly the silent corruption this run exists to
            // catch, so one of those is a red result however few there are.
            self.ticks > 0
                && self.read_failures == 0
                && self.bad_gyro == 0
                && self.bad_gravity == 0
                && self.bad_joints == 0
                && self.ready_after.is_some()
        }

        fn print(&self) {
            println!();
            println!(
                "ticks          {:>6}   {:.1} Hz",
                self.ticks,
                self.ticks as f64 / self.seconds
            );
            println!("read failures  {:>6}", self.read_failures);
            println!(
                "stale ticks    {:>6}{}",
                self.stale_ticks,
                if self.stale_ticks == 0 {
                    String::new()
                } else {
                    format!(
                        "   ({:.2}% held the previous attitude)",
                        100.0 * self.stale_ticks as f64 / self.ticks.max(1) as f64
                    )
                }
            );
            println!(
                "last gyro      {:+.3} {:+.3} {:+.3} rad/s   (module axes, unrotated)",
                self.last_gyro[0], self.last_gyro[1], self.last_gyro[2]
            );
            println!(
                "last gravity   {:+.3} {:+.3} {:+.3}   |q| = {:.4}",
                self.last_gravity[0], self.last_gravity[1], self.last_gravity[2], self.last_norm
            );
            match self.ready_after {
                Some(n) => println!("ready after    {n:>6} ticks"),
                None => println!("ready after         -   never converged"),
            }

            println!();
            println!("observation slot checks");
            println!("  obs[0..3]   gyro        {} mismatch(es)", self.bad_gyro);
            println!(
                "  obs[3..6]   gravity     {} mismatch(es)",
                self.bad_gravity
            );
            println!(
                "  obs[6..34]  joints      {} non-zero (want 0)",
                self.bad_joints
            );

            println!();
            if self.passed() {
                println!("PASS — a real IMU reaches the policy's slots intact.");
                println!("Only the servo bus is left unproven, and it is not on this bench.");
            } else if self.bad_gyro > 0 || self.bad_gravity > 0 {
                println!("FAIL — the observation disagrees with the IMU. The layout is wrong;");
                println!("       do not trust anything downstream of it.");
            } else if self.bad_joints > 0 {
                println!("FAIL — the joint block is not zero though the joints are at home.");
            } else if self.read_failures > 0 {
                println!("FAIL — the bus dropped reads.");
            } else {
                println!("FAIL — the orientation never converged.");
            }
        }
    }

    fn drive<B: RegisterBlocks>(mut io: ImuOnlyIo<B>, window: Duration) -> Report {
        let mut report = Report {
            seconds: 0.0,
            ticks: 0,
            read_failures: 0,
            stale_ticks: 0,
            bad_gyro: 0,
            bad_gravity: 0,
            bad_joints: 0,
            ready_after: None,
            last_gyro: [0.0; 3],
            last_gravity: [0.0; 3],
            last_norm: 0.0,
        };

        // A still robot is told to stand, which is the command a bench can justify: no twist,
        // no head, nominal stance.
        let command = Command::default();
        let last_action = [0.0f32; ACTION_LEN];
        let mut previous_quat = None;

        let start = Instant::now();
        let deadline = start + window;
        while Instant::now() < deadline {
            let tick = Instant::now();

            let sensors = match io.read() {
                Ok(sensors) => sensors,
                Err(e) => {
                    report.read_failures += 1;
                    if report.read_failures <= 3 {
                        eprintln!("read failed: {e}");
                    }
                    std::thread::sleep(PERIOD);
                    continue;
                }
            };
            report.ticks += 1;

            // A held attitude is the decoder doing its job, but it is worth counting: it is the
            // one degradation the loop above cannot see, because a stale orientation looks
            // exactly like a still robot.
            if previous_quat == Some(sensors.imu.quat) {
                report.stale_ticks += 1;
            }
            previous_quat = Some(sensors.imu.quat);

            let observation = Observation::build(
                &sensors.imu,
                &sensors.positions,
                &sensors.velocities,
                &DEFAULT_POSITION,
                &last_action,
                &command,
            );
            let slots = observation.as_slice();

            // The two checks this example exists for. Tolerances are f32-sized: the
            // observation is `[f32; 61]` and the IMU is f64, so the round trip is lossy by
            // design and comparing exactly would fail on every tick.
            for i in 0..3 {
                if (f64::from(slots[i]) - sensors.imu.gyro[i]).abs() > 1e-6 {
                    report.bad_gyro += 1;
                }
                if (f64::from(slots[3 + i]) - sensors.imu.gravity[i]).abs() > 1e-6 {
                    report.bad_gravity += 1;
                }
            }
            if slots[6..34].iter().any(|v| v.abs() > 1e-6) {
                report.bad_joints += 1;
            }

            report.last_gyro = sensors.imu.gyro;
            report.last_gravity = sensors.imu.gravity;
            report.last_norm = sensors.imu.quat.iter().map(|c| c * c).sum::<f64>().sqrt();

            if report.ready_after.is_none() && io.imu_ready() {
                report.ready_after = Some(report.ticks);
            }

            if report.ticks.is_multiple_of(PRINT_EVERY) {
                println!(
                    "tick {:>5}  gyro [{:+.3} {:+.3} {:+.3}]  gravity [{:+.3} {:+.3} {:+.3}]",
                    report.ticks, slots[0], slots[1], slots[2], slots[3], slots[4], slots[5],
                );
            }

            // The policy's own output fed back. On a bench nothing drives it, so it stays
            // zero — but the observation is built through the real entry point rather than a
            // shortened copy of it, which is the whole reason this exists.
            let spent = tick.elapsed();
            if spent < PERIOD {
                std::thread::sleep(PERIOD - spent);
            }
        }
        report.seconds = start.elapsed().as_secs_f64();
        report
    }
}
