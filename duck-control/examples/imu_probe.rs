//! Read the external IMU module off its I²C bus and report whether it is fit to run a robot.
//!
//! This is the bench tool for the swap, and the only step that needs hardware at all — and it
//! needs *only* the module. No servos, no assembled robot. Run it before wiring anything else.
//!
//! ```text
//! cargo run -p duck-control --example imu_probe             # find the module, read for 20 s
//! cargo run -p duck-control --example imu_probe -- /dev/i2c-3 30
//! ```
//!
//! It answers, in the order that would stop you:
//!
//! * does the module answer, and on which bus
//! * is it producing samples at the rate the control loop needs — 50 Hz, and the module does 100
//! * are the samples *new*. A module repeating one frame looks fast and is not, which is why
//!   the new-value rate is reported next to the sample rate rather than instead of it
//! * does the decoded attitude make sense: gravity is a unit vector near −Z when the module is
//!   level, and the gyro sits near zero when it is still — except that this module's stillness
//!   gate zeroes it *exactly*, which is a property, not a fault
//! * what a sample costs on the bus, which is the number that decides whether this can share a
//!   tick with the servos
//!
//! It does **not** calibrate the mount, and cannot: the mount is a fact about how the module is
//! bolted to the trunk, not about the bus. Until that is measured this reports attitude in the
//! module's own axes, so a level module need not read as level.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The IMU module is reached through /dev/i2c-*, which exists only on Linux.");
    eprintln!("Run this on the robot, or on any Linux board with the module attached.");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() {
    probe::run();
}

#[cfg(target_os = "linux")]
mod probe {
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use duck_control::imu_i2c::{ADDRESS, Icm42670pI2c, RegisterBlocks, open_first};
    use duck_control::imu_icm42670p::{GYRO_RAD_PER_LSB, Icm42670pDecoder, Icm42670pSample};

    /// The module fuses at 100 Hz; pacing the reads to that is what the robot will do, so the
    /// probe measures the same thing the robot will live with rather than hammering the bus.
    const RATE_HZ: u64 = 100;
    const PERIOD: Duration = Duration::from_micros(1_000_000 / RATE_HZ);

    /// Without a mount there is no rotation to apply, and inventing one would make the report
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
                eprintln!();
                eprintln!("Find the bus rather than guessing it — the numbering is per board:");
                eprintln!(
                    "  i2cdetect -l                 which buses exist, and what they are called"
                );
                eprintln!("  sudo i2cdetect -y -r N       should show {ADDRESS:#04x}");
                eprintln!("then name it:");
                eprintln!("  cargo run -p duck-control --example imu_probe -- /dev/i2c-N 20");
                eprintln!();
                eprintln!("Opening /dev/i2c-* needs root or the `i2c` group, and the bus only");
                eprintln!(
                    "exists once I²C is enabled — raspi-config on a Pi, jetson-io on a Jetson."
                );
                std::process::exit(1);
            }
        };

        println!("reading for {seconds:.0} s at {RATE_HZ} Hz...");
        let report = measure(module, Duration::from_secs_f64(seconds));
        report.print();
        if !report.passes() {
            std::process::exit(1);
        }
    }

    struct Report {
        seconds: f64,
        samples: u64,
        fresh: u64,
        failures: u64,
        rejected: u64,
        bus_nanos: u128,
        ready_after: Option<u64>,
        last: Option<(Icm42670pSample, f64, [f64; 3])>,
    }

    impl Report {
        fn sample_hz(&self) -> f64 {
            self.samples as f64 / self.seconds
        }

        /// How often the whole frame changed, which is not the same as how often the module
        /// produced something. Gyro noise makes every frame differ, so this sits at 100% by
        /// construction and only moves when a frame is frozen *entire* — the failure it is here
        /// to catch. Whether the quaternions inside those frames are usable is
        /// [`Self::rejected`]'s question, and this number cannot answer it.
        fn fresh_hz(&self) -> f64 {
            self.fresh as f64 / self.seconds
        }

        /// What one sample costs on the bus — the share of a 20 ms tick it would claim.
        fn bus_ms(&self) -> f64 {
            if self.samples == 0 {
                return 0.0;
            }
            self.bus_nanos as f64 / self.samples as f64 / 1e6
        }

        fn passes(&self) -> bool {
            self.failures == 0 && self.fresh_hz() >= 45.0 && self.ready_after.is_some()
        }

        fn print(&self) {
            println!();
            println!(
                "samples        {:>6}   {:.1} Hz",
                self.samples,
                self.sample_hz()
            );
            println!(
                "frame changes  {:>6}   {:.1} Hz   (catches a frozen frame, nothing subtler)",
                self.fresh,
                self.fresh_hz()
            );
            println!("read failures  {:>6}", self.failures);
            println!(
                "rejected quats {:>6}{}",
                self.rejected,
                if self.rejected == 0 {
                    String::new()
                } else {
                    format!(
                        "   <- {:.2}% of samples held the previous attitude",
                        100.0 * self.rejected as f64 / self.samples.max(1) as f64
                    )
                }
            );
            println!("bus time       {:>6.2} ms per sample", self.bus_ms());
            match self.ready_after {
                Some(n) => println!("ready after    {n:>6} samples"),
                None => {
                    println!("ready after         -   the module never produced 25 quaternions")
                }
            }
            if let Some((raw, norm, gravity)) = self.last {
                println!();
                println!(
                    "last gyro      {:+.3} {:+.3} {:+.3} rad/s   (module axes, unrotated)",
                    raw.gyro[0] as f64 * GYRO_RAD_PER_LSB,
                    raw.gyro[1] as f64 * GYRO_RAD_PER_LSB,
                    raw.gyro[2] as f64 * GYRO_RAD_PER_LSB,
                );
                println!(
                    "last gravity   {:+.3} {:+.3} {:+.3}   |q| = {norm:.4}",
                    gravity[0], gravity[1], gravity[2]
                );
            }

            println!();
            if self.passes() {
                println!("PASS — the module is fit to wire into robotd.");
                println!(
                    "Next: measure the mount. Three poses (level, +90° about X, +90° about Y),"
                );
                println!("and the quaternion that carries one to the other is the mount constant.");
            } else if self.fresh_hz() < 45.0 {
                println!("FAIL — the module is repeating whole frames.");
                println!(
                    "A frozen frame is what a module whose output rate was never raised looks"
                );
                println!("like from here: set it over the serial link and measure again.");
            } else if self.failures > 0 {
                println!(
                    "FAIL — the bus dropped reads. Check wiring and pull-ups before anything else."
                );
            } else {
                println!("FAIL — the module never produced a usable quaternion.");
            }
        }
    }

    fn measure<B: RegisterBlocks>(mut module: Icm42670pI2c<B>, window: Duration) -> Report {
        let mut decoder = Icm42670pDecoder::new(UNROTATED);
        let mut report = Report {
            seconds: 0.0,
            samples: 0,
            fresh: 0,
            failures: 0,
            rejected: 0,
            bus_nanos: 0,
            ready_after: None,
            last: None,
        };
        let mut previous: Option<Icm42670pSample> = None;

        let start = Instant::now();
        let deadline = start + window;
        while Instant::now() < deadline {
            let tick = Instant::now();
            match module.read_sample() {
                Ok(sample) => {
                    report.bus_nanos += tick.elapsed().as_nanos();
                    report.samples += 1;
                    if previous != Some(sample) {
                        report.fresh += 1;
                    }
                    previous = Some(sample);
                    let decoded = decoder.decode(&sample);
                    if report.ready_after.is_none() && decoder.ready() {
                        report.ready_after = Some(report.samples);
                    }
                    let norm = decoded.quat.iter().map(|c| c * c).sum::<f64>().sqrt();
                    report.last = Some((sample, norm, decoded.gravity));
                }
                Err(e) => {
                    report.failures += 1;
                    // Say it the first few times and then stop: a module that is unplugged
                    // would otherwise fill the terminal with the same line.
                    if report.failures <= 3 {
                        eprintln!("read failed: {e}");
                    }
                }
            }
            let spent = tick.elapsed();
            if spent < PERIOD {
                std::thread::sleep(PERIOD - spent);
            }
        }
        report.seconds = start.elapsed().as_secs_f64();
        // Every successful read was decoded once, so the gap between the reads and the
        // quaternions the decoder kept is exactly the ones it refused.
        report.rejected = report.samples - u64::from(decoder.quat_samples());
        report
    }
}
