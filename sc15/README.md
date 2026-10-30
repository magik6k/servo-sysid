# Waveshare SC15: unit 1

Waveshare SC15, a Feetech SCS15-compatible serial-bus servo, bus ID 1.
Measured 2026-10-28 to 2026-10-30 to fit the actuator and sensor models
of the brobt RL simulator.

**Charts:** [`rendered/README.md`](rendered/README.md), regenerated with
`python3 sc15/render.py`.

## Bench

| Part | Detail |
|---|---|
| Controller | M5 Atom Lite (ESP32), USB `/dev/ttyUSB0` (FTDI). Flash with FQBN `esp32:esp32:m5stack_atom`, core ≥ 3.3.11, **`UploadSpeed=115200`** (faster rates fail in ftdi_sio). Under heavy servo current the USB adapter can re-enumerate while the ESP32 keeps running; `chaos_log.py` now reopens the port and carries on |
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
| 1.52kg-50mm | The same spool at the first hole, 50 mm | 0.76 N·m | 2026-10-30, probed heavy batch |

The spool also spins with the arm: its own inertia is roughly
0.004 kg·m², assuming wire between ~26 and 70 mm radius (to be
measured), on top of 0.0064 kg·m² from hanging at 65 mm.

## Protocol and registers

SCS protocol, big-endian 16-bit registers. These are the registers
used, with values read back from this unit:

| Reg | Meaning | Value / use |
|---|---|---|
| 0x09–0x0C | Angle limits | 0/1023 = position mode; **0/0 = motor mode** (continuous) |
| 0x0D / 0x0E / 0x0F | Max temperature / max / min voltage | 80 °C / 9.0 V / 4.5 V: the error-bit limits |
| 0x10 | Max torque | 1000 (so 1000 = 100 % PWM) |
| 0x13 | Unload mask | 0x24 = overheat (bit 2) and overload (bit 5) act; voltage (bit 0) only flags |
| 0x25 / 0x26 / 0x27 | Probably protective torque / protection time / overload torque, as on Feetech STS | 20 / 100 / 80. The overload clamp holds the output at exactly 200, so 0x25 = 20 % fits; the trigger does not match an 80 % threshold |
| 0x15 / 0x16 | Position loop P / D | 15 / 15 by default. The firmware sets P per block (8, 15, 30) |
| 0x1A / 0x1B | Dead band | 1 / 1 |
| 0x28 | Torque enable | |
| 0x2A–0x2F | Goal position, goal time (ms), goal speed | Goal time 1 ms (with speed 0) = flat out |
| 0x2C | Motor-mode PWM | 0..1000, bit 10 = direction |
| 0x30 | Lock | Reads 1 at power-up, so EEPROM-area writes (mode limits) apply but are **not saved**. A power cycle restores position mode |
| 0x38–0x3F | Position, speed, load, voltage, temperature | Load is the PWM actually applied, signed like the command |
| Error byte | Status in every reply | Bit 0 = supply outside 4.5–9.0 V; torque is not cut. Bit 5 = overload: output clamped to 200 (see Torque). Bit 2 = overheat |

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
| Position-mode output at rest | **30 + kp × (\|error\| − 1)** units exactly at kp 8, 15 and 30: punch (0x18–19 = 30) plus P past the 1-count dead band. 0 within ±2 counts (±1 at kp 30). No integral | heavy batch |
| Position-mode law, moving | u = −sign(e)(30 + kp(\|e\| − 1)) + kd · speed register / 250, kd ≈ 72–75 at every kp, clipped ±1000, then the slew. Predicts the logged output to 9–18 units on slow trajectories, 40–140 on fast chirps | trajectory blocks |
| Goal time | The internal goal ramps linearly to a new goal over the goal time: 600 ms moves take ~600–700 ms | holds |

### Speed and sensing

| Property | Value | Source |
|---|---|---|
| Speed vs PWM, arm only | Linear, no dead band: 150 → 11 RPM, 400 → 31, 600 → 47, 800 → 62, 1000 → 73–76. Directions within ~3 % | arm-only laps |
| Steady speed constant | ≈ 2.12 counts/s per PWM unit (unloaded, ~8.5 V) | unloaded chaos |
| Mechanical time constant | ≈ 50 ms (weak fit, R² 0.5), small next to the slew | unloaded chaos |
| Position sensor | 0..1023 spans **~227°** (≈ 1610 counts/rev; the manual says 200°) | arm-only laps, 25 laps, two methods within 1.3 % |
| Blind spot | ~133° (37 % of a turn): reads pinned at 0 or 1022, speed reads 0 or garbage | laps |
| Gap crossing | The reading sweeps the whole range in 1–3 samples (e.g. 1018 → 222 → 48 → 0). Trust 20..1000 only, and reject steps > 30 counts/sample | laps |
| Speed register | Count change from 7 to 2 ticks back over 20 ms (≈ 18 ms old), quantized to 50 counts/s. Tracks dpos/dt with slope 0.986, r 0.991 | unloaded chaos, trajectory blocks |

### Torque, gearbox, inertia

| Property | Value | Source |
|---|---|---|
| Torque per PWM unit, lifting | 3.26 (cold) to 3.58 (warm) mN·m at 3–9 % output; 2.5–2.7 at 15–25 %; **2.4 at a 35 % stall**; 2.33 in slow lifts at 50 mm and 1.92 at 65 mm (cold, higher output). Falls with power, so a linear extrapolation (3.3 N·m at 100 %) is wrong | holds, 100 g laps, heavy turns, heavy holds |
| Heavy turns, 0.76 N·m (50 mm) | 50, 65 and 80 % turn even warm; 35 % stalls | probed batch |
| Heavy turns, 0.98 N·m | Cold (26 °C): 35 % stalls at 56–66° (0.81–0.90 N·m); 50 % lifts past horizontal at 35–60°/s. **Warm (38 °C reported), from rest: 35 % stalls at 44°, 50 % at 67°, 65 % at 94°**; only 80 % turns | heavy first turns, heavy batch |
| Thermal derating | Stall torque per unit ~20 % lower at 38 °C than at 26 °C reported; 25 % probes at 50 mm: 2.2–2.5 → 1.8 mN·m from 26 to 46 °C, and 1.66 right after laps + bang at a reported 38 °C (the sensor lags the winding). Copper alone gives ~5 % for +12 °C, so the winding runs much hotter than the sensor. Bang cycles also slowed from 0.56–0.66 s to 0.68–0.72 s warm. The sim needs a thermal state | heavy batch |
| Gearbox efficiency | Forward × back ≈ 0.30 cold, 0.43 warm with 100 g; 0.25 with 1.52 kg (holds fit, which also puts down 19 counts off, so treat it loosely). Lowering still needs the motor to push. Needs a directional friction model in sim | holds, heavy holds |
| Friction band | At 45° with 1.52 kg (0.69 N·m) the arm stays put while the drive varies 16–30 %. It lifts above ~30 % and lowers below ~11 % (ratio ≈ 0.37) | heavy batch backlash block |
| Push to move at ~16°/s | ≈ 43 PWM units: ~34 back-EMF plus ~10 Coulomb | holds |
| Self-holding | Torque off, 0.13 N·m: never falls, even at ±104°. With 1.52 kg, released anywhere up to ±99°: falls back in 1.5–2 s (peak ~100°/s) and **stops at ±8.9° at 65 mm and +10.5°/−12° at 50 mm: 0.14–0.16 N·m static friction either way, independent of the load**. At ~1 N·m, 0 % PWM in motor mode: creeps back ~2°/s | lift-drop (100 g and 1.52 kg), heavy turns |
| Holding cost | In position mode the P term keeps pushing on a held load (e.g. 28.5 % at +96° with 1.52 kg) although the gearbox would hold it at zero | full climb |
| Hunting at the bottom | No gravity preload: the position loop hunts ±2–3 counts with 100 g, ±6–8 counts (±1.3–1.8°) and ±15–17 % output with 1.52 kg at kp 15. At kp 8 it mostly settles | holds, gentle steps, backlash block |
| Backlash | Not separable with this sensor: the pot is on the output, so gear play shows only as hunting and as the friction band, never as a position offset. Needs a motor-side reference (or a stall peg and a torque sensor) | backlash block |
| Droop under load | Position holds settle 3–4.5° short of the goal with 1.52 kg at kp 15; 3.1° at kp 15 and 6° at kp 8 at 45°, exactly what the output law needs to carry the load | heavy swings, backlash block |
| Reflected inertia | Unknown. If the 50 ms time constant is real, ≈ 0.01–0.03 kg·m². An earlier "7e-5" was a bad fit and is retracted | unloaded chaos |

### Tracking with 1.52 kg

Position-mode trajectories at 65 mm (and 50 mm, last rows), valid frames only. Lag is the
cross-correlation peak between goal and position.

| Trajectory | kp 8 | kp 15 | kp 30 |
|---|---|---|---|
| raise_lower (12.5°/s to ±100°) | 7.8° RMS, 328 ms | 4.6°, 192 ms | 2.7°, 96 ms |
| chirp_soft (45°, 0.2 → 1.0 Hz) | 11.8°, 84 ms | 6.2°, 48 ms | 5.2°, 36 ms |
| dual_sine_soft (60° at 0.15 Hz + 8° at 2 Hz) | 7.5°, 92 ms | 5.4°, 60 ms | 4.3°, 44 ms |
| *50 mm:* raise_lower | 5.2°, 240 ms | 3.0°, 128 ms | 1.9°, 68 ms |
| *50 mm:* chirp_soft | 10.3°, 84 ms | 6.1°, 48 ms | 6.3°, 40 ms |
| *50 mm:* dual_sine_soft | 6.7°, 88 ms | 4.6°, 56 ms | 3.4°, 36 ms |

Output saturates in at most 1.5 % of frames (dual-sine at kp 30), so the
soft trajectories stay mostly inside the actuator's authority.

### Supply

| Property | Value | Source |
|---|---|---|
| Torque vs supply | **Proportional**: cold 25 % probes give 257 µN·m per unit per volt from 4.6 to 8.6 V. So output torque ∝ duty × supply, as the sim model assumes. The 7.4 V run sits ~8 % low, inside the run-to-run spread: a repeat at 8.7 V gave cold probes ~10 % below the first (2.08 vs 2.32 mN·m), converging by 45 °C | voltage runs at 50 mm, 8.7 V repeat |
| Loaded speed vs supply | 80 % laps with 1.52 kg at 50 mm: 58 / 42 / 35 RPM at 8.7 / 7.4 / 6.6 V, a line reaching zero at 2.4 V of duty × supply; at 4.8 V every level stalls. Load and friction take a fixed share of the voltage | voltage runs at 50 mm |
| Low supply | At 4.8 V (below the rated 6 V) the supply sagged to 3.4 V with no servo reset; error bit 0 flags under- as well as overvoltage and doesn't cut torque | 4.8 V run |
| Sag | 8.7 V at rest → 7.9–8.2 V at full output while moving → **6.5–6.9 V** at full output near standstill (bang, raise-lower at kp 30, chaos) | full climb, bang, heavy batch |
| Regen | A falling heavy mass back-drives the motor beyond its free speed. **18 V** on a scope without a clamp; ~+2 V with the diode clamp | heavy turns, regen check |
| Voltage register | Never reads above 9.9 V, so it likely clips: logs understate spikes | all |
| Temperature | ≤ 34 °C in every run before the heavy batch, which reached **51 °C** and was still rising. The register has single-frame glitches (27↔30, and spikes up to 62 °C), so filter it | all |

### Protection and failure when hot

From the hot runs at the end of 2026-10-30, 1.52 kg at 50 mm, 8.7 V. They
ended in damage to unit 1 (last row).

| Property | Value | Source |
|---|---|---|
| Overload (error bit 5) | Trips only in position-mode stalls short of the goal (holds at ±100°). The output is then held at exactly 200 (0x25 = 20 %) until a different goal arrives, and clears within ~0.1 s. The trigger is unclear: 360 and 390 at 7.1–8.4 V tripped after 1–2 s, while 330 for 4.4 s and 660 for 4.6 s did not. Pushing into an obstacle can quietly drop to 20 % | 7.4 V run, 8.7 V repeat |
| Overheat (error bit 2) | Not a latch at 80 °C. Under load, single frames read exactly 100 °C with bit 2 and cut the output to 0 for one tick, so the ramp restarts from 0. Rare from 57 °C, dense at 74–75 °C reported, where the output stays near 0. None at all with torque off at the same temperature: the over-limit readings come with motor current | burn, burn_high |
| Heating | Full-duty chaos: 45 → 73 °C in 25 min, heading for ~80–90 °C with τ ≈ 20 min. It averages only ~50 % output because of the slew limit. A narrow bang near horizontal (output saturated at low speed) heats ~5× faster, ~5 °C/min at 62–66 °C. Hanging with torque off: 75 → 64 °C in 3.5 min | burn, burn_high, hot batches |
| Torque when hot | 25 % probes: 1.6–1.8 mN·m per unit at 47 °C, 1.3–1.5 at 66 °C, about −1 %/°C (2.1–2.5 cold) | burn |
| Silent drive dropout | At ~72 °C reported, about 75 s after a ~6 s full-output stall at 73 °C, the motor drew no current for over 70 s. The load register still reported up to ±1000 and no error bit was set. It worked again at 62 °C. Only the missing supply sag and motion show it | hot batch |
| Damage | **The output gear stripped in one spot.** It first shows 570 s into the hot batch: full-output stalls at +54° and −48°, then binding at +62° and −41° in the next run. Gravity couldn't back-drive it there either, and backing off it moved stop-go. Nothing like it appears in any earlier run, including the 25 min burn. Most likely cause: 1.52 kg slammed through reversals and braking at 70–75 °C (burn_high, heat blocks). Unit 1 is retired | hot batches |

## Run log

All raw runs are in [`data/`](data): `<run>.csv.gz` plus `<run>.meta.txt`,
which holds the firmware preamble, plan, register dump and any notes. The
CSV columns are documented in `shared/tools/chaos_log.py`. Unit state
and the last trusted time per run: [`data/runs.csv`](data/runs.csv). Block splits
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
| `chaos_20261030_093448_heavy-batch-1.52kg-at-65mm` | 1.52kg | Full heavy batch: holds, backlash kp 15/8, soft trajectories at kp 8/15/30, lift-drop, heavy laps, bang, 300 s chaos | Good up to 694 s. Logger died on a USB dropout 146 s into chaos; the board finished unlogged. Heavy laps 35–65 % stalled warm |
| `chaos_20261030_094731_heavy-batch-1.52kg-at-65mm-tail` | 1.52kg | The rest of that boot: last 87 s of chaos, then the park | Good. Join to the main part on segment numbers; ~68 s of chaos is lost |
| `chaos_20261030_101658_heavy-batch-probed-1.52kg-at-50mm` | 1.52kg-50mm | Probed heavy batch at 8.7 V: 25 % probe ×5, plus the full heavy batch | Good. No dropout, all probes stalled. Supply turned to 6.6 V during the park |
| `chaos_20261030_105115_heavy-batch-probed-1.52kg-at-50mm-6.6V` | 1.52kg-50mm | The same probed batch at a 6.6 V supply, from 27 °C | Good. Sagged to 5.0 V; 35 and 50 % laps stalled |
| `chaos_20261030_112510_heavy-batch-probed-1.52kg-at-50mm-7.4V` | 1.52kg-50mm | The same probed batch at a 7.4 V supply, from 27 °C | Good. Sagged to 5.6 V; 35 and 50 % laps stalled |
| `chaos_20261030_115756_heavy-batch-probed-1.52kg-at-50mm-4.8V` | 1.52kg-50mm | The same probed batch at a 4.8 V supply (below spec), from 27 °C | Good. Sagged to 3.4 V without resets; every lap level stalled; many position stall-guard trips |
| `chaos_20261030_134414_heavy-batch-burn-1.52kg-at-50mm-8.7V` | 1.52kg-50mm | The probed batch again at 8.7 V from 22 °C, then `burn`: full-duty chaos with a probe every minute | Batch good (two USB dropouts, reconnected). Burn 45 → 73 °C over 25 min, stopped by request before any trip |
| `chaos_20261030_142508_burn-high-1.52kg-at-50mm-8.7V-from-hot` | 1.52kg-50mm | `burn_high` (brakes flat out while falling through the lower half) from 74 °C | Overheat throttling from ~59 s, trip at 59.8 s, then 3.5 min of cooldown hanging |
| `chaos_20261030_143007_hot-batch-1.52kg-at-50mm-8.7V` | 1.52kg-50mm | `HOT_BATCH`: the probed batch with heat blocks keeping it at 69–72 °C | Valid to 570 s. Gear damage from 570 s (full-output stalls at +54° and −48°), silent drive dropout from 700 s |
| `chaos_20261030_145017_hot-batch-bang-heat-1.52kg-at-50mm-8.7V` | 1.52kg-50mm | `HOT_BATCH` with the bang heater, 63–66 °C, cutoff 69 °C | Gearbox binding at +62° and −41°. Stopped at 218 s; failure data only |

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
- Trusting a single temperature reading: it glitches by up to +15 °C for
  one frame. The firmware's 65 °C cutoff now needs 3 readings in a row.
- Comparing torque across runs without the temperature: a warm motor
  gives ~20 % less. Even at the same temperature, runs differ by ~10 %.
- Running the SC15 above ~65 °C reported, or stalling it at full output
  when hot. Unit 1 lost its drive at 72 °C and then its gearbox bound.
  The stall guard doesn't prevent a persistent stall either: it turns
  torque back on with the next segment.

## Model

BAM-style fit in [`model/`](model/README.md). The firmware controller is
identified exactly, plus the motor, BAM M1–M6 friction and a thermal
factor. Validation on two held-out runs: **0.81° (0.0141 rad)** under
the BAM protocol, against 0.0093–0.0095 rad for BAM's best fits on the
Dynamixels. The heavy pendulum at kp 8–15 reaches 0.47–0.56°, which is at
BAM's level. Open-loop motor mode with a 1 s resync scores 3–4.5°.

## Open questions

- The overload trigger, and the stick-slip at kp 30 (steps of 3–5°)
  that the friction models don't produce.
- How to split back-EMF from viscous friction: only 11 torque-off drop
  windows.
- How big the backlash is. The output-side pot can't show it; needs a
  motor-side reference or a stall peg plus a torque sensor.
- Temperature dependence: efficiency rose from 0.30 to 0.43 warm with
  100 g, while stall torque fell ~20 % warm with 1.52 kg. A thermal model
  needs torque against temperature at fixed load, cold to hot.
- Why the USB adapter drops under load (ground return through USB?). A
  USB isolator or a separate ground path would tell.
- Is the supply sag a current limit or resistance? Watch it on the scope.
- Spread between units: needs a second SC15.
- Stall torque at 100 %, which needs a stop peg and care with the gears.

## Next runs

Unit 1 is retired (stripped output gear). The data from it is complete
up to the hot runs. What a second unit should add, at ≤ 65 °C:

1. **The BAM grid**: masses 0.3–1.5 kg at 50–150 mm, kp 4/8/16/32,
   chirp, dual-sine, raise-lower and many lift-drops. The light setups and
   the drops are what this data lacks.
2. **The overload trigger**: position stalls against a hard stop at a
   ladder of outputs and durations.
3. **A current shunt** on the servo supply: heat input for a thermal
   model, and a supply-sag model.
4. **An output encoder** (≥ 12 bit) for ground truth past the pot's
   0.22° counts and its hunting.
5. **Unit-to-unit spread**: the same probed batch on each new unit.

Recording, from the repo root:

    arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp32:esp32:m5stack_atom:UploadSpeed=115200 shared/firmware/chaos
    python3 shared/tools/chaos_log.py --seconds <s> --tag <setup>-<what>
    python3 shared/tools/split_batch.py sc15/data/chaos_<stamp>_<tag>.csv.gz
    python3 sc15/render.py
