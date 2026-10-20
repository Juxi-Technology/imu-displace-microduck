//! Capture the module's attitude in poses the operator declares.
//!
//! ## What a mount is, and why this cannot be finished on a bare bench
//!
//! The mount is the rotation that carries the module's frame to the trunk's. It is a fact
//! about **how the module is bolted to the trunk**, so it cannot be measured from the module
//! alone: with no trunk there is nothing for the rotation to be relative to, and a bench run
//! can only say what the module's own axes are.
//!
//! What this tool does now is the part that *is* available before the trunk exists — capture
//! the module's frame cleanly, in whatever orientation the operator holds it, with a check
//! that the module did not move during the window. Mounted later, the same capture in the
//! robot's standing pose plus a declaration of which module axis points forward is the whole
//! of the mount.
//!
//! ```text
//! cargo run -p duck-control --example imu_mount                      # live, 2 Hz
//! cargo run -p duck-control --example imu_mount -- --capture level   # average 5 s, save
//! cargo run -p duck-control --example imu_mount -- /dev/i2c-7 --capture tilt-x --seconds 8
//! ```
//!
//! ## Reading the live output
//!
//! ```text
//! gravity   +0.016 -0.010 -1.000        |q| 1.0000
//!             ^      ^      ^
//!           module module module
//!             X      Y      Z
//! ```
//!
//! Gravity is the world's **down** expressed in the module's frame — `ImuData` documents
//! upright as `[0, 0, -1]` — so the vector names the module axis that points down. Held flat
//! with the component side up it reads near `+0.0 +0.0 −1.0`, meaning the module's `−Z` is
//! down and its `+Z` is the component-side normal. Measured 2026-10-15.
//!
//! ## Why a capture is averaged rather than sampled once
//!
//! One sample carries the gyro's noise and the module's own fusion wobble. An average over a
//! few seconds does not, and the deviation from that average is the number that says whether
//! the operator's hand was steady enough for the result to mean anything — a capture that
//! moved is worse than no capture, because it looks like data.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The IMU module is reached through /dev/i2c-*, which exists only on Linux.");
    eprintln!("Run this on the robot, or on any Linux board with the module attached.");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() {
    mount::run();
}

#[cfg(target_os = "linux")]
mod mount {
    use std::io::Write as _;
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use duck_control::imu_i2c::{Icm42670pI2c, RegisterBlocks, open_first};
    use duck_control::imu_icm42670p::{GYRO_RAD_PER_LSB, Icm42670pDecoder, Icm42670pSample};

    /// Faster than the control loop, because a capture averages anyway and a pose the operator
    /// is holding by hand drifts.
    const RATE_HZ: u64 = 100;
    const PERIOD: Duration = Duration::from_micros(1_000_000 / RATE_HZ);

    /// Live readout cadence. A human is watching, not a log.
    const PRINT_EVERY: u64 = 50;

    const UNROTATED: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    /// A capture whose deviation from its own mean exceeds this moved, and is not a pose.
    /// Sized from the module's own noise: a still module's gravity wanders by a few
    /// thousandths, so a tenth is well clear of it and still catches a real movement.
    const STEADY_TOLERANCE: f64 = 0.1;

    pub fn run() {
        let mut bus: Option<PathBuf> = None;
        let mut label: Option<String> = None;
        let mut seconds = 5.0f64;
        let mut settle = 3.0f64;
        let mut file: Option<PathBuf> = None;

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--capture" => {
                    label = Some(
                        args.next()
                            .unwrap_or_else(|| die("--capture wants a label")),
                    )
                }
                "--seconds" => {
                    seconds = args
                        .next()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or_else(|| die("--seconds wants a number"));
                }
                "--settle" => {
                    settle = args
                        .next()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or_else(|| die("--settle wants a number"));
                }
                "--file" => {
                    file = Some(PathBuf::from(
                        args.next().unwrap_or_else(|| die("--file wants a path")),
                    ))
                }
                other => match other.parse::<f64>() {
                    Ok(secs) => seconds = secs,
                    Err(_) => bus = Some(PathBuf::from(other)),
                },
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
                std::process::exit(1);
            }
        };
        let mut decoder = Icm42670pDecoder::new(UNROTATED);

        match label {
            Some(label) => capture(module, &mut decoder, &label, seconds, settle, file),
            None => live(module, &mut decoder, seconds),
        }
    }

    fn die(message: &str) -> ! {
        eprintln!("{message}");
        std::process::exit(2);
    }

    fn sample_or_warn<B: RegisterBlocks>(module: &mut Icm42670pI2c<B>) -> Option<Icm42670pSample> {
        match module.read_sample() {
            Ok(sample) => Some(sample),
            Err(e) => {
                eprintln!("read failed: {e}");
                None
            }
        }
    }

    /// Show the attitude as it comes, so a human can watch the effect of moving the module.
    fn live<B: RegisterBlocks>(
        mut module: Icm42670pI2c<B>,
        decoder: &mut Icm42670pDecoder,
        seconds: f64,
    ) {
        println!("live, {RATE_HZ} Hz for {seconds:.0} s — module axes, no mount applied");
        println!("gravity points up in the module's frame; |q| should sit near 1");
        println!();

        let deadline = Instant::now() + Duration::from_secs_f64(seconds);
        let mut ticks = 0u64;
        while Instant::now() < deadline {
            let tick = Instant::now();
            if let Some(sample) = sample_or_warn(&mut module) {
                ticks += 1;
                if ticks.is_multiple_of(PRINT_EVERY) {
                    let imu = decoder.decode(&sample);
                    let norm = imu.quat.iter().map(|c| c * c).sum::<f64>().sqrt();
                    println!(
                        "gravity   {:+.3} {:+.3} {:+.3}        |q| {norm:.4}   gyro {:+.2} {:+.2} {:+.2}",
                        imu.gravity[0],
                        imu.gravity[1],
                        imu.gravity[2],
                        sample.gyro[0] as f64 * GYRO_RAD_PER_LSB,
                        sample.gyro[1] as f64 * GYRO_RAD_PER_LSB,
                        sample.gyro[2] as f64 * GYRO_RAD_PER_LSB,
                    );
                }
            }
            pace(tick);
        }
    }

    /// Average one pose and record it.
    fn capture<B: RegisterBlocks>(
        mut module: Icm42670pI2c<B>,
        decoder: &mut Icm42670pDecoder,
        label: &str,
        seconds: f64,
        settle: f64,
        file: Option<PathBuf>,
    ) {
        println!(
            "capturing '{label}': {settle:.0} s to get into position, then {seconds:.0} s held."
        );
        println!();

        // The operator starts the run and only then has their hands where they want them — so
        // the reach into the pose is discarded rather than averaged in. Without this, a capture
        // taken while the module was still being placed reads as one that moved, which is
        // indistinguishable from a hand that shook.
        let deadline = Instant::now() + Duration::from_secs_f64(settle + seconds);
        let settled_at = Instant::now() + Duration::from_secs_f64(settle);
        let mut quats: Vec<[f64; 4]> = Vec::new();
        let mut gravity: Vec<[f64; 3]> = Vec::new();
        let mut failures = 0u64;

        while Instant::now() < deadline {
            let tick = Instant::now();
            match module.read_sample() {
                Ok(sample) => {
                    // Decoded on every sample, including the settle, and only *recorded* after
                    // it. Skipping the decode during the settle would leave the decoder's
                    // histories cold, and its first output is then the initial state rather
                    // than a measurement — one sample at right angles to the pose, which the
                    // spread check reads as a hand that moved. It cost four captures to find,
                    // and only because the pose held happened to differ from upright.
                    let imu = decoder.decode(&sample);
                    if Instant::now() >= settled_at {
                        quats.push(imu.quat);
                        gravity.push(imu.gravity);
                    }
                }
                Err(e) => {
                    failures += 1;
                    if failures <= 3 {
                        eprintln!("read failed: {e}");
                    }
                }
            }
            pace(tick);
        }

        if gravity.len() < 10 {
            die("too few samples to average — is the module answering?");
        }

        let mean_quat = mean_quaternion(&quats);
        let mean_gravity = mean_vector(&gravity);
        let spread = gravity
            .iter()
            .map(|g| distance(*g, mean_gravity))
            .fold(0.0f64, f64::max);
        let norm = mean_quat.iter().map(|c| c * c).sum::<f64>().sqrt();

        println!(
            "captured '{label}': {} samples over {seconds:.0} s, {failures} read failures",
            gravity.len()
        );
        println!(
            "  quaternion  w {:+.4}  x {:+.4}  y {:+.4}  z {:+.4}   |q| {norm:.4}",
            mean_quat[0], mean_quat[1], mean_quat[2], mean_quat[3]
        );
        println!(
            "  gravity     {:+.4} {:+.4} {:+.4}",
            mean_gravity[0], mean_gravity[1], mean_gravity[2]
        );
        println!("  spread      {spread:.4}  (max deviation over the window)");

        if spread > STEADY_TOLERANCE {
            println!();
            println!("  !! the module moved during the window — this is not one pose.");
            println!("     Nothing was saved: a pose that moved is worse than no pose, because");
            println!("     it looks like data and a solver downstream cannot tell.");
            println!("     Hold it still and run the same capture again.");
            std::process::exit(1);
        }

        let path = file.unwrap_or_else(default_capture_file);
        match append_capture(&path, label, mean_quat, mean_gravity, spread) {
            Ok(()) => println!("  appended to {}", path.display()),
            Err(e) => {
                eprintln!("  could not write {}: {e}", path.display());
                std::process::exit(1);
            }
        }

        println!();
        println!("The mount still needs a trunk. Mounted, capture the robot's standing pose as");
        println!("'level' and declare which module axis points forward; that plus the robot");
        println!("rotated 90° about each trunk axis is the whole of it.");
    }

    fn default_capture_file() -> PathBuf {
        match std::env::var_os("HOME") {
            Some(home) => PathBuf::from(home).join("imu-mount.txt"),
            None => PathBuf::from("imu-mount.txt"),
        }
    }

    /// One line per capture, human-readable and easy to hand-edit. Not a format anything
    /// parses yet — a solver that reads it will pin it then, and until there is a trunk there
    /// is nothing to solve.
    fn append_capture(
        path: &PathBuf,
        label: &str,
        quat: [f64; 4],
        gravity: [f64; 3],
        spread: f64,
    ) -> std::io::Result<()> {
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)?;
        writeln!(
            file,
            "{label}\tquat {:.6} {:.6} {:.6} {:.6}\tgravity {:.6} {:.6} {:.6}\tspread {spread:.6}",
            quat[0], quat[1], quat[2], quat[3], gravity[0], gravity[1], gravity[2]
        )
    }

    /// Averaged with the sign ambiguity removed.
    ///
    /// `q` and `−q` are the same rotation, so a module sitting exactly on the boundary between
    /// them would average to zero — and the boundary is not exotic, it is wherever the fused
    /// yaw happens to cross. Each sample is therefore flipped into the first sample's
    /// hemisphere before it is summed.
    fn mean_quaternion(samples: &[[f64; 4]]) -> [f64; 4] {
        let reference = samples[0];
        let mut sum = [0.0f64; 4];
        for q in samples {
            let dot: f64 = (0..4).map(|i| q[i] * reference[i]).sum();
            let sign = if dot < 0.0 { -1.0 } else { 1.0 };
            for i in 0..4 {
                sum[i] += sign * q[i];
            }
        }
        let norm = sum.iter().map(|c| c * c).sum::<f64>().sqrt();
        if norm <= 0.0 {
            return reference;
        }
        [sum[0] / norm, sum[1] / norm, sum[2] / norm, sum[3] / norm]
    }

    fn mean_vector(samples: &[[f64; 3]]) -> [f64; 3] {
        let mut sum = [0.0f64; 3];
        for v in samples {
            for i in 0..3 {
                sum[i] += v[i];
            }
        }
        let n = samples.len() as f64;
        [sum[0] / n, sum[1] / n, sum[2] / n]
    }

    fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
        (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
    }

    fn pace(tick: Instant) {
        let spent = tick.elapsed();
        if spent < PERIOD {
            std::thread::sleep(PERIOD - spent);
        }
    }
}
