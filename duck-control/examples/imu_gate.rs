//! Find the stillness gate: how slowly can the module turn before it stops reporting it.
//!
//! The module zeroes its **raw gyro** on all three axes while it believes it is still. That is
//! harmless on a bench and potentially not harmless on a robot: an angular velocity of exactly
//! zero is not noise, it is a dropped observation, and the policy's two IMU inputs are the gyro
//! and projected gravity.
//!
//! The difficulty is that the gate destroys the measurement of its own threshold — a zeroed
//! output says nothing about how fast the module was actually turning. So this measures the
//! true rate **independently, from the fused quaternion**: differentiate it over time and the
//! rotation is there whether or not the gyro reports it.
//!
//! Which also answers the second question. If the quaternion keeps tracking a slow turn while
//! the gyro reads zero, the gate is on the *output* and the fusion still sees the motion. If
//! the quaternion stops too, the motion is being lost before it is fused, and every slow
//! movement the robot makes is invisible to the policy.
//!
//! ```text
//! cargo run -p duck-control --example imu_gate -- /dev/i2c-7 40
//! ```
//!
//! ## How to run it
//!
//! Start with the module still, then **turn it slowly** — as slowly as you can — and gradually
//! increase the speed over the run, holding each speed for a second or two so the bins fill.
//! Any axis will do. The interesting part of the table is where the "exactly zero" column
//! changes, and the bins either side of it need samples to say anything.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The IMU module is reached through /dev/i2c-*, which exists only on Linux.");
    eprintln!("Run this on the robot, or on any Linux board with the module attached.");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() {
    gate::run();
}

#[cfg(target_os = "linux")]
mod gate {
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use duck_control::imu_i2c::{Icm42670pI2c, RegisterBlocks, open_first};
    use duck_control::imu_icm42670p::Icm42670pDecoder;

    const RATE_HZ: u64 = 100;
    const PERIOD: Duration = Duration::from_micros(1_000_000 / RATE_HZ);
    const PRINT_EVERY: u64 = 50;
    const UNROTATED: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    /// Upper edges of the speed bins, in °/s. Dense where a threshold is likely to sit — a
    /// stillness gate is about a few degrees per second, not about tens.
    const EDGES: [f64; 7] = [0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0];

    pub fn run() {
        let mut bus: Option<PathBuf> = None;
        let mut seconds = 40.0f64;
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
                std::process::exit(1);
            }
        };

        println!("{RATE_HZ} Hz for {seconds:.0} s.");
        println!("Still first, then turn it slowly and keep speeding up. The rate below is");
        println!("measured from the quaternion, so it is independent of the gyro output.");
        println!();

        let report = drive(module, Duration::from_secs_f64(seconds));
        report.print();
    }

    fn drive<B: RegisterBlocks>(mut module: Icm42670pI2c<B>, window: Duration) -> Report {
        let mut decoder = Icm42670pDecoder::new(UNROTATED);
        let mut report = Report::default();

        let mut previous: Option<([f64; 4], Instant)> = None;
        let deadline = Instant::now() + window;
        while Instant::now() < deadline {
            let tick = Instant::now();
            let now = Instant::now();
            match module.read_sample() {
                Ok(sample) => {
                    report.samples += 1;
                    let imu = decoder.decode(&sample);
                    let raw_zero = sample.gyro.iter().all(|&g| g == 0);

                    // A repeated quaternion is either a genuine still moment or a sample the
                    // decoder refused — in both cases the derivative is meaningless, and
                    // counting it would stuff the slowest bin with intervals that were never
                    // measured.
                    if let Some((last_quat, last_at)) = previous {
                        let dt = now.duration_since(last_at).as_secs_f64();
                        if imu.quat != last_quat && dt > 0.0 {
                            let rate =
                                rates_between(last_quat, imu.quat).map(|w| magnitude(w) / dt);
                            if let Some(rate) = rate {
                                report.record(rate, raw_zero);
                                if report.samples.is_multiple_of(PRINT_EVERY) {
                                    println!(
                                        "  raw gyro {:>6}   true rate {:>7.2} °/s",
                                        if raw_zero { "zero" } else { "live" },
                                        rate.to_degrees()
                                    );
                                }
                            } else {
                                report.derivative_failures += 1;
                            }
                        } else {
                            report.skipped += 1;
                        }
                    } else {
                        report.skipped += 1;
                    }
                    previous = Some((imu.quat, now));
                }
                Err(e) => {
                    report.read_failures += 1;
                    if report.read_failures <= 3 {
                        eprintln!("read failed: {e}");
                    }
                }
            }
            let spent = tick.elapsed();
            if spent < PERIOD {
                std::thread::sleep(PERIOD - spent);
            }
        }
        report
    }

    /// The body-frame angular velocity that carries `from` to `to`, in radians per second
    /// *before* the elapsed time is divided out — so callers scale by their own `dt`.
    ///
    /// `q` and `−q` are the same rotation; taking the shorter arc keeps a derivative that
    /// crosses that boundary from reading as a full turn.
    fn rates_between(from: [f64; 4], to: [f64; 4]) -> Option<[f64; 3]> {
        let dot: f64 = (0..4).map(|i| from[i] * to[i]).sum();
        let sign = if dot < 0.0 { -1.0 } else { 1.0 };
        let to = [to[0] * sign, to[1] * sign, to[2] * sign, to[3] * sign];

        // dq = to · from⁻¹, the rotation from one to the other in the same frame.
        let inv = [from[0], -from[1], -from[2], -from[3]];
        let dq = [
            to[0] * inv[0] - to[1] * inv[1] - to[2] * inv[2] - to[3] * inv[3],
            to[0] * inv[1] + to[1] * inv[0] + to[2] * inv[3] - to[3] * inv[2],
            to[0] * inv[2] - to[1] * inv[3] + to[2] * inv[0] + to[3] * inv[1],
            to[0] * inv[3] + to[1] * inv[2] - to[2] * inv[1] + to[3] * inv[0],
        ];

        let axis = [dq[1], dq[2], dq[3]];
        let sin_half = magnitude(axis);
        if sin_half < 1e-12 {
            return Some([0.0; 3]);
        }
        // 2·atan2(|v|, w): stable where acos(w) is not, and `w` is already ≥ 0 from the flip.
        let angle = 2.0 * sin_half.atan2(dq[0]);
        let scale = angle / sin_half;
        Some([axis[0] * scale, axis[1] * scale, axis[2] * scale])
    }

    fn magnitude(v: [f64; 3]) -> f64 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }

    #[derive(Default)]
    struct Report {
        samples: u64,
        read_failures: u64,
        skipped: u64,
        derivative_failures: u64,
        /// Per bin: (samples, samples whose raw gyro was exactly zero).
        bins: [Option<(u64, u64)>; EDGES.len()],
        /// The fastest true rate seen with a zeroed gyro, and the slowest seen live.
        fastest_zero: f64,
        slowest_live: Option<f64>,
    }

    impl Report {
        fn record(&mut self, rate_rad: f64, raw_zero: bool) {
            let dps = rate_rad.to_degrees();
            // One bin per interval: the highest edge the rate clears. Filling every edge below
            // it would make the table cumulative and hide exactly the transition it exists to
            // show.
            if let Some(index) = EDGES.iter().rposition(|&edge| dps >= edge) {
                let (total, zero) = self.bins[index].get_or_insert((0, 0));
                *total += 1;
                if raw_zero {
                    *zero += 1;
                }
            }
            if raw_zero {
                self.fastest_zero = self.fastest_zero.max(dps);
            } else {
                self.slowest_live = Some(match self.slowest_live {
                    Some(v) => v.min(dps),
                    None => dps,
                });
            }
        }

        fn print(&self) {
            println!();
            println!(
                "samples {samples}, read failures {failures}",
                samples = self.samples,
                failures = self.read_failures
            );
            println!(
                "intervals skipped {skipped} (repeated quaternion), derivative failures {failures}",
                skipped = self.skipped,
                failures = self.derivative_failures
            );
            println!();
            println!("true rate bin, °/s      intervals   raw gyro exactly zero");
            println!("------------------------------------------------------------");
            for (index, &edge) in EDGES.iter().enumerate() {
                let upper = EDGES.get(index + 1).copied();
                let label = match upper {
                    Some(u) => format!("{edge:.1} – {u:.1}"),
                    None => format!("{edge:.1} and faster"),
                };
                match self.bins[index] {
                    Some((total, zero)) => println!(
                        "{label:<22} {total:>9}   {zero:>6}  ({:.0}%)",
                        100.0 * zero as f64 / total as f64
                    ),
                    None => println!("{label:<22} {0:>9}   {0:>6}", 0),
                }
            }
            println!();
            println!(
                "fastest turn with the gyro zeroed : {:.2} °/s",
                self.fastest_zero
            );
            match self.slowest_live {
                Some(v) => println!("slowest turn with the gyro live   : {:.2} °/s", v),
                None => println!("slowest turn with the gyro live   : never — it never came alive"),
            }
            println!();
            println!("If the zeroed column falls to 0% at a rate well above the module's noise,");
            println!("that rate is the gate. If the quaternion followed the slow turns (the bins");
            println!("filled at all), the gate is on the output and the fusion still saw them.");
        }
    }
}
