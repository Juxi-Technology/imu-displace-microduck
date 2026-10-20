# Replacing the trunk IMU with an I²C module (ICM-42670-P)

Swap the trunk IMU from the `imu_to_dxl` board — which rides the servo bus pretending to be
Dynamixel ID 200 — to an off-the-shelf module on I²C.

Written for **HD-1910 builds**. It touches the IMU only; **the servo protocol is unchanged**.

> **Status**: the IMU data path is finished and verified on real hardware (sampling, decoding,
> observation slots). **The runtime (`robotd`) is not wired up yet** — see
> [Wiring it into the runtime](#wiring-it-into-the-runtime). What works today is the bench
> tooling.

---

## Why it is not "wire it to the bus and answer as ID 200"

ID 200 is a device number on the **Dynamixel bus**, not an I²C address. They are different
physical layers and different protocols:

| | Servo bus | I²C |
|---|---|---|
| Physical layer | single-wire half-duplex TTL | two wires, SDA/SCL |
| Rate | 1 Mbps | 100 / 400 kHz |
| Protocol | Dynamixel Protocol V2 | register reads and writes |
| Identity | ID (0–253) | slave address (7 bit) |
| Device | `/dev/ttyS2` | `/dev/i2c-N` |

**An I²C module on the HAT's I²C header is `0x23` and stays `0x23`.** Nothing translates that
into ID 200. To appear on the bus as a servo you would need an MCU in between re-implementing
`imu_to_dxl`'s timing — which is not worth it.

Instead the host reads **two buses separately**: servos on the servo bus, the IMU over I²C, and
merges them into `Sensors`.

That buys more than skipping a custom board: **it separates the failure domains**. In the
original arrangement, an IMU board that fails to answer invalidates the whole `sync_read` and
all fifteen servos lose their data for that tick. Apart, a dead IMU means no attitude while the
joints carry on.

---

## Hardware

| | |
|---|---|
| Module | ICM-42670-P, I²C slave **0x23** |
| Wiring | **GND / 3V3 / SDA / SCL** |
| Supply | **3V3**. Not 5V — the host's I/O is 3.3 V |
| Bus | HAT: `/dev/i2c-pihat` (the overlay's symlink) = `/dev/i2c-3`; `/dev/i2c-4` after moving J6's R18/R19 to R16/R17 |
| Address conflicts | J5 already carries 0x18 (audio), 0x19/0x68 or 0x6A (onboard IMU), 0x29 (ToF) — **0x23 collides with none** |

Enabling the bus: the HAT needs the `i2c3-pihat` overlay (at the cost of USB-C PD negotiation);
Raspberry Pi uses `raspi-config`; Jetson uses `jetson-io.py`.

```bash
sudo apt install -y i2c-tools
i2cdetect -l
sudo i2cdetect -y -r <bus>
```

Continue only when `0x23` appears. If it does not, that is wiring, power or a wrong bus number —
not the code.

---

## Quick verification

Three layers, in order. **Only the module is needed; no servos.**

```bash
# 1. Transport: rate, dropped reads, bus time
cargo run -p duck-control --example imu_probe -- /dev/i2c-7 20

# 2. Observation wiring: does the IMU land in the six slots the policy reads
cargo run -p duck-control --example imu_to_observation -- /dev/i2c-7 20

# 3. Pose capture (for calibration)
cargo run -p duck-control --example imu_mount -- /dev/i2c-7
cargo run -p duck-control --example imu_mount -- /dev/i2c-7 --capture level
```

**Expected** (Jetson Orin Nano + `/dev/i2c-7`, measured):

| | |
|---|---|
| Sample rate | 99.4 Hz, no read failures |
| Bus time | 0.92 ms per sample |
| `rejected quats` | 1–2% |
| Step 2 | `obs[0..3]` / `obs[3..6]` / joint blocks **zero mismatches** |

---

## Wiring it into the runtime

The `duck-control` side is ready; **the `robotd` side is not written**.

| | |
|---|---|
| `DynamixelIo::open_without_bus_imu()` — stops asking for id 200 | ✅ |
| `ExternalImuIo::new(joints, module, mount)` — merges into `Sensors` | ✅ |
| Config keys (bus path / mount / source selection) | ⛔ needs `robotd-params` + `deploy/robotd.toml` |
| `robotd` choosing the construction path from config | ⛔ still calls `DynamixelIo::open` unconditionally |
| IMU thread + shared slot | ⛔ **`ExternalImuIo` reads synchronously**, about 1 ms per tick |
| Startup self-check | ⛔ `imu_answers()` returns `false` in the no-board mode; the startup check must follow |

Code layout:

| File | Purpose |
|---|---|
| `imu_icm42670p.rs` | decode: module registers → `ImuData` (gyro ±2000 dps, float32 quaternion with `w`) |
| `imu_i2c.rs` | transport: `RegisterBlocks` trait, register map, bus scan |
| `imu_external.rs` | merge: `ExternalImuIo` |
| `bus.rs` | `open_without_bus_imu()` |

---

## Calibrating the mount

The mount is the rotation from **module frame to trunk frame**. It cannot be determined without
the trunk — there is nothing for the rotation to be relative to.

Reference mounting: the module's axes coincide with the trunk's (**X forward / Y left / Z up**),
which physically means **component side up, Type-C aft**. That gives
**mount = `[1, 0, 0, 0]`**.

The module's own frame (measured, three orthogonal poses):

| Pose | gravity (the world's *down* in module axes) |
|---|---|
| Flat, component side up | `[0, 0, -1]` |
| Type-C up | `[1, 0, 0]` |
| On edge, component face right | `[0, 1, 0]` |

⇒ flat: **+X points away from the Type-C end, +Y to your right, +Z up** (component-face normal).

`imu_mount` discards the first 3 s of a capture, averages the next 5, and reports **spread** —
the largest deviation inside the window. Above 0.1 it refuses to save, because **a pose that
moved is worse than no pose: it looks like data.**

> ⚠️ A different mounting needs a new mount. **Being 90° out passes every electrical check**;
> the symptom is only that the duck falls the wrong way.

---

## Acceptance checklist (once fitted)

| Check | Pass |
|---|---|
| Standing | gravity ≈ `[0, 0, -1]` |
| Rotate 90° about trunk X | ≈ `[0, ±1, 0]` |
| Rotate 90° about trunk Y | ≈ `[±1, 0, 0]` |
| Gyro signs | correct on X for a roll right, on Y for a nod down |
| 50 Hz period | not slowed by the I²C read |
| Continuous run | zero read failures |

**The two axis checks are not optional**: gravity is blind to yaw, and a yaw error swaps roll
and pitch rates while every electrical check still passes.

---

## Known behaviour (not faults)

| Symptom | Explanation |
|---|---|
| **About 1–2 reads in 100 return an all-zero block** | The module clears the block before writing its fused result, an ~80 µs window. The decoder's norm guard rejects it and holds the previous sample. **Do not remove that check** — without it an all-zero quaternion reads as "the robot is upright". The rate is **independent of rotation speed** (1–3% at every speed) |
| **The raw gyro reads exactly 0 when still** | A stillness gate, not a dead sensor. **Its threshold is far below anything the robot does**: the gyro reports at a true rate of 0.14 °/s, and 97% of intervals below 0.5 °/s are live — **slow motion is not clipped** |
| **It runs at 100 Hz out of the box** | The protocol sheet says 25 Hz; measured 100 Hz. The setting is stored in flash, so check a new module |
| **The quaternion norm is about 0.998** | That is what the module gives; the decoder normalises |
| The first sample after start-up is the initial state | The median needs two priors; exact from the second sample. `ready()` is false for the first 25 |

---

## Not verified yet

| | Why |
|---|---|
| The bus going from 16 blocks to 15 | Needs a real bus with fifteen servos |
| The runtime wiring | Not implemented |
| **Whether the policy still walks** | Needs a complete robot. A different IMU changes the observation distribution, which the simulation does not model |

---

## Rolling back

Every change is inside `duck-control`, and the bus keeps its original path: construct with
`DynamixelIo::open()` as before and the IMU is on the bus again. Nothing else changes.

---

## Measurement record

| | Value | Environment |
|---|---|---|
| Unit tests | 111 passed | Windows + aarch64 |
| Sample rate / dropped reads | 99.4 Hz / 0 | Jetson Orin Nano + `/dev/i2c-7` |
| Bus time | 0.92 ms per sample (bus inferred at 400 kHz from timing) | same |
| Rejected quaternions | 1.3 / 1.7 / 1.8% (100 Hz); 0.10% (50 Hz loop) | same |
| Stillness gate | live at 0.14 °/s; 97% live below 0.5 °/s | same, `imu_gate`, 3975 samples |
