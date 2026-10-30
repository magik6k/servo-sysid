# Waveshare SC15: unit 1

Waveshare SC15, a Feetech SCS15-compatible serial-bus servo, bus ID 1.
Measured 2026-10-28 to 2026-10-30 to fit the actuator and sensor models
of the brobt RL simulator (`../../docs/control-interface.md`).

**Charts:** [`rendered/README.md`](rendered/README.md), regenerated with
`python3 sc15/render.py`.

## Bench

| Part | Detail |
|---|---|
| Controller | M5 Atom Lite (ESP32), USB `/dev/ttyUSB0` (FTDI). Flash with FQBN `esp32:esp32:m5stack_atom`, core ≥ 3.3.11, **`UploadSpeed=115200`** (faster rates fail in ftdi_sio) |
| Servo bus | One-wire half-duplex UART on G33 at 1 Mbaud: same pin for RX and TX (open-drain), external pull-up to 3.3 V. Bus ID 1 |
| Supply | Bench supply at ~8.7 V. Since 2026-10-30 a stack of 30–40 diodes across the servo supply clamps regen (~50 mA at 8.7 V, ~+2 V spikes) |
| Stand | `shared/rig/pendulum_rig.scad`: 165 mm axis height, 4× M3 into the horn (14 mm square), 4 mm of print under the screw heads |
| Arm | Printed, 13.5 g, CoM 61.5 mm, J 7.3e-5 kg·m², M8 holes at 50/65/80/95/110/125 mm. The heavy arm as printed is ~20 g (user's estimate) |
| Down | The arm hangs straight down at **count 513** (from the holds fit; mounted visually at 511) |

Reading the board: open the port with DTR/RTS low and pulse RTS to
reset it. The firmware prints CRLF lines. `arduino-cli monitor` shows
nothing non-interactively.

### Setups

| Name in `servo.json` | Load | Peak gravity torque | Runs |
|---|---|---|---|
| unloaded | Horn only | 0 | 2026-10-28 chaos |
| arm | 13.5 g arm | 0.008 N·m | 2026-10-29 laps, chaos |
| 100g | 100.4 g at 125 mm + arm | 0.131 N·m | 2026-10-29 holds, batch, trajectories |
| 1.52kg | 1.52 kg wire spool (with hub, clamp, bolt) at 65 mm + heavy arm (~20 g) | 0.98 N·m | 2026-10-30 onward |

The spool also spins with the arm: its own inertia is roughly
0.004 kg·m², assuming wire between ~26 and 70 mm radius (to be
measured), on top of 0.0064 kg·m² from hanging at 65 mm.

## Protocol and registers

SCS protocol, big-endian 16-bit registers. These are the registers
used, with values read back from this unit:

| Reg | Meaning | Value / use |
|---|---|---|
| 0x09–0x0C | Angle limits | 0/1023 = position mode; **0/0 = motor mode** (continuous) |
| 0x10 | Max torque | 1000 (so 1000 = 100 % PWM) |
| 0x15 / 0x16 | Position loop P / D | 15 / 15 by default. The firmware sets P per block (8, 15, 30) |
| 0x1A / 0x1B | Dead band | 1 / 1 |
| 0x28 | Torque enable | |
| 0x2A–0x2F | Goal position, goal time (ms), goal speed | Goal time 1 ms (with speed 0) = flat out |
| 0x2C | Motor-mode PWM | 0..1000, bit 10 = direction |
| 0x30 | Lock | Reads 1 at power-up, so EEPROM-area writes (mode limits) apply but are **not saved**. A power cycle restores position mode |
| 0x38–0x3F | Position, speed, load, voltage, temperature | Load is the PWM actually applied, signed like the command |
| Error byte | Status in every reply | Bit 0 = overvoltage (> 9.0 V). Torque is not cut (unload mask 0x24) |

Full register dumps are in every run's `.meta.txt`.

## What we know

The charts for most of these rows are in [`rendered/README.md`](rendered/README.md).

### Timing and command path

| Property | Value | Source |
|---|---|---|
| Internal update | 4 ms (250 Hz); output changes in one-tick steps | unloaded chaos |
| Command → output starts changing | ≤ 1 tick in motor mode. Usually 1 tick, worst ~50 ms, in position mode | unloaded chaos |
| Telemetry read failures | 0.14 %. Hold the last value and flag it | unloaded chaos |
| Streaming goals | Write the goal **after** that tick's telemetry read. Writing it right before the read lost 58–96 % of reads | batch vs trajectories runs |

### Modes and output slew

| Property | Value | Source |
|---|---|---|
| Positive PWM | Drives the position count **down** | all |
| Motor-mode slew | **1.25 units/ms exactly**, both directions: 0 → full in 0.8 s, full reversal 1.6 s. Not set by goal speed, torque cycling, or any register found | laps |
| Motor-mode entry (writing 0/0 limits) | Resets the output to 0 at once **and clears the PWM register**, so the duty must be written after the switch | laps, bounce tests |
| "Bounce" | Writing position limits and then motor limits resets the ramp to 0: full reversal in 0.8 s instead of 1.6 s | bounce sweep |
| Position-mode entry | Resets the goal to the present position, so write the goal after the switch | holds |
| Position-mode slew | **Asymmetric.** Output collapses through zero at ~100 units/ms on a reversal, then builds at ~5 units/ms (0 → full 0.2 s) | bang test |
| Position steps, unloaded | Within 10 counts after ~169 ms + 0.46 ms/count. ~0 overshoot, ±1 count final error | unloaded chaos |

### Speed and sensing

| Property | Value | Source |
|---|---|---|
| Speed vs PWM, arm only | Linear, no dead band: 150 → 11 RPM, 400 → 31, 600 → 47, 800 → 62, 1000 → 73–76. Directions within ~3 % | arm-only laps |
| Steady speed constant | ≈ 2.12 counts/s per PWM unit (unloaded, ~8.5 V) | unloaded chaos |
| Mechanical time constant | ≈ 50 ms (weak fit, R² 0.5), small next to the slew | unloaded chaos |
| Position sensor | 0..1023 spans **~227°** (≈ 1610 counts/rev; the manual says 200°) | arm-only laps, 25 laps, two methods within 1.3 % |
| Blind spot | ~133° (37 % of a turn): reads pinned at 0 or 1022, speed reads 0 or garbage | laps |
| Gap crossing | The reading sweeps the whole range in 1–3 samples (e.g. 1018 → 222 → 48 → 0). Trust 20..1000 only, and reject steps > 30 counts/sample | laps |
| Speed register | = dpos/dt in counts/s (slope 0.986, r 0.991), quantized to 50 | unloaded chaos |

### Torque, gearbox, inertia

| Property | Value | Source |
|---|---|---|
| Torque per PWM unit, lifting | 3.26 (cold) to 3.58 (warm) mN·m at 3–9 % output; 2.5–2.7 at 15–25 %; **2.4 at a 35 % stall**. Falls with power, so a linear extrapolation (3.3 N·m at 100 %) is wrong | holds, 100 g laps, heavy turns |
| Heavy turns, 0.98 N·m | 35 % stalls at 56–66° (0.81–0.90 N·m); 50 % lifts past horizontal at 35–60°/s | heavy first turns |
| Gearbox efficiency | Forward × back ≈ 0.30 cold, 0.43 warm (≈ 0.55–0.66 each way). Lowering still needs the motor to push. Needs a directional friction model in sim | holds |
| Push to move at ~16°/s | ≈ 43 PWM units: ~34 back-EMF plus ~10 Coulomb | holds |
| Self-holding | Torque off, 0.13 N·m: never falls, even at ±104°. At ~1 N·m, 0 % PWM in motor mode: creeps back ~2°/s | lift-drop (100 g), heavy turns |
| Holding cost | In position mode the P term keeps pushing on a held load (e.g. 28.5 % at +96° with 1.52 kg) although the gearbox would hold it at zero | full climb |
| Hunting at the bottom | No gravity preload: the position loop hunts ±2–3 counts with 100 g, ±6 counts (±1.3°) and ±15 % output with 1.52 kg. Backlash still to measure | holds, gentle steps |
| Droop under load | Position holds settle 3–4.5° short of the goal with 1.52 kg at kp 15 | heavy swings |
| Reflected inertia | Unknown. If the 50 ms time constant is real, ≈ 0.01–0.03 kg·m². An earlier "7e-5" was a bad fit and is retracted | unloaded chaos |

### Supply

| Property | Value | Source |
|---|---|---|
| Sag | 8.7 V at rest → 7.9–8.2 V at full output while moving → **6.9 V** at full output near standstill (bang test) | full climb, bang |
| Regen | A falling heavy mass back-drives the motor beyond its free speed. **18 V** on a scope without a clamp; ~+2 V with the diode clamp | heavy turns, regen check |
| Voltage register | Never reads above 9.9 V, so it likely clips: logs understate spikes | all |
| Temperature | ≤ 34 °C in every run so far | all |

## Run log

All raw runs are in [`data/`](data): `<run>.csv.gz` plus `<run>.meta.txt`,
which holds the firmware preamble, plan, register dump and any notes. The
CSV columns are documented in `shared/tools/chaos_log.py`. Block splits
of v2 runs are regenerated into `data/derived/` (not in git) with
`shared/tools/split_batch.py`.

| Run | Setup | What | Status |
|---|---|---|---|
| `chaos_20261028_173504_unloaded` | unloaded | chaos v1: 240 s random segments (75 % motor, 25 % position) | Bounced motor segments never drove (duty written before the mode switch). Use `load`, not `command`, as the input |
| `chaos_20261029_180324_arm-only-laps` | arm | laps v1 | **Dead**: never drove (same bug) |
| `chaos_20261029_181040_arm-only-laps` | arm | laps v1: constant PWM 15–100 %, both directions | Good; the sensor-span calibration |
| `chaos_20261029_182307_arm-13.5g` | arm | chaos v1, 240 s | Good |
| `chaos_20261029_184653_holds-100.4g-at-125mm` | 100g | holds v1: 10° steps to ±100°, up then down | Good; gearbox efficiency, true down |
| `chaos_20261029_190523_batch-100.4g-at-125mm` | 100g | v2 batch: chaos 300 s, laps, holds, chirp/dual-sine/raise-lower at kp 8/15/30, lift-drop | Trajectory blocks lost most reads; use the next run for those |
| `chaos_20261029_192200_trajectories-100.4g-at-125mm` | 100g | chirp, dual-sine, raise-lower at kp 8/15/30 | Good (BAM-style identification data) |
| `holdsteps_20261030_003328_gentle-heavy-first` | 1.52kg | ±10°, ±20° position steps at 20 Hz (`hold_position` sketch) | Good; hunting at the bottom |
| `chaos_20261030_004203_heavy-first-turns` | 1.52kg | swings to ±105°, heavy laps 35–80 % | Good; no clamp, regen to 18 V |
| `chaos_20261030_011704_regen-check-clamp-1.52kg-at-65mm` | 1.52kg | swings, 50 % laps, with the clamp | Good; −50 % stalled at horizontal |
| `chaos_20261030_012121_regen-soak-3x80pct-clamp-1.52kg-at-65mm` | 1.52kg | three 5 s bursts of 80 % laps | Good |
| `chaos_20261030_012614_full-climb-1.52kg-at-65mm` | 1.52kg | one flat-out move from down to +100° | Good |
| `chaos_20261030_013021_bang-reversals-1.52kg-at-65mm` | 1.52kg | 3 flat-out cycles between ~30° and 102°, kp 30 | Good |

## Mistakes to avoid

- Writing the duty before entering motor mode: the switch clears it.
- Writing a goal just before reading telemetry: the read fails.
- Fitting holds at rest: the gearbox holds by friction, so the rest PWM
  says nothing about torque. Fit the slow move phases, split by direction
  (`shared/tools/holds_analyze.py`).
- Differentiating raw position: gap sweeps look like valid in-range
  readings. Use the trust window plus the step filter
  (`servolab.add_kinematics`).
- Using quasi-static regressions at speed with a heavy mass: friction,
  inertia and back-EMF are entangled. That needs the full model fit.
- Running overhauling loads without a supply clamp.

## Open questions

- A full model fit, BAM-style (simulation-based, MuJoCo): k_t, R, k_e,
  directional and load-dependent friction, reflected inertia.
- How big the backlash is, and whether the wobble comes from the gearbox
  or the mount. The next run's backlash block measures it.
- Temperature dependence: efficiency went from 0.30 to 0.43 warm.
- Is the supply sag a current limit or resistance? Watch it on the scope.
- Spread between units: needs a second SC15.
- Stall torque at 100 %, which needs a stop peg and care with the gears.

## Next run: full heavy batch

The firmware plan `HEAVY_BATCH` in `shared/firmware/chaos/chaos.ino` (it
is already the selected `PLAN`) runs about 14.5 min plus any cooldown:

| Block | Gain | Time | Purpose |
|---|---|---|---|
| holds | 15 | 105 s | Efficiency and torque per unit at ~7× the 100 g load |
| backlash | 15, 8 | 2 × 53 s | 1/2/4/8° steps out and back, hanging down (no preload) and at 45° (preloaded) |
| chirp_soft | 8, 15, 30 | 3 × 22.5 s | 45°, 0.2 → 1.0 Hz (the 60°/1.5 Hz chirp asks for more torque than exists) |
| dual_sine_soft | 8, 15, 30 | 3 × 22.5 s | 60° at 0.15 Hz + 8° at 2.0 Hz |
| raise_lower | 8, 15, 30 | 3 × 36.5 s | 12.5°/s ramps to ±100° |
| lift_drop | 15 | 40 s | Torque off at ±30/60/90/105°; at ~1 N·m it may now fall |
| heavy_laps | 15 | 57.5 s | 35/50/65/80 %, both directions, ramped stops |
| bang | 30 | ~13 s | 3 full-speed reversal cycles near horizontal |
| chaos | 15 | 300 s | Random segments. Last, because it heats the servo most |

The stall guard (1.5 s, motor ≥ 15 % or position ≥ 80 %) now ends the
segment with torque off instead of stopping the run. A motor-mode
equilibrium can therefore sit for most of a chaos segment, which gives
static torque data. The thermal cutoff (65 °C off, 50 °C on) pauses the
plan.

Checklist:

1. Spool on the 65 mm hole and tight. Base clamped to the bench.
2. Diode clamp connected. Supply at 8.7 V with the current limit raised
   to ≥ 3 A. Watch for the CC indicator during the bang block.
3. Flash:
   `arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp32:esp32:m5stack_atom:UploadSpeed=115200 shared/firmware/chaos`
4. Record, from the repo root (resets the board and starts the plan):
   `python3 shared/tools/chaos_log.py --seconds 1200 --tag heavy-batch-1.52kg-at-65mm`
5. Split it and update the charts:
   `python3 shared/tools/split_batch.py sc15/data/chaos_<stamp>_heavy-batch-1.52kg-at-65mm.csv.gz`,
   then `python3 sc15/render.py`.
