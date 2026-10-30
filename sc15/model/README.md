# SC15 actuator model

Simulation-based fit of the SC15 on the pendulum bench, following BAM
(Duclusaud et al., "Extended Friction Models for the Physics Simulation of
Servo Actuators", ICRA 2025). Code in `shared/tools`:

- `sc_model.py`: the model (numba)
- `model_data.py`: windows
- `fit_model.py`: CMA-ES fit and validation

Charts are in [`../rendered/`](../rendered/README.md#model).

| File | Fit |
|---|---|
| `bam-m1.json` … `bam-m6.json` | BAM protocol: trajectory and lift-drop windows, friction models M1–M6 |
| `full-m6.json` | Full model: every window kind, M6 friction, thermal factor, kd held at 74.5 |

## Structure, per 4 ms tick

| Part | Model | Source |
|---|---|---|
| Sensor | count = round(513 + θ·1610/2π); the blind spot reads 1022 or 0. Speed register = (c[k−2] − c[k−7]) / 20 ms, quantized to 50 counts/s | laps, register fit |
| Goal | Linear ramp to a new goal over the goal time; mode entry resets it to the present position | holds moves |
| Position law | u* = −sign(e)·(30 + kp(\|e\| − 1)) for \|e\| > 2 (> 1 at kp 30), + kd · speed / 250, clipped ±1000 | output fit: 9–18 units on slow trajectories |
| Output slew | Position mode: up 20/tick, down 400/tick. Motor mode: ±5/tick. Mode entry: 0 | slew tests |
| Motor | τ = f(T)·(k·u·V − b·ω), u = −load/1000, V = measured supply, f(T) = 1/(1 + aT(T − 25)). Torque off: τ = 0 | |
| Friction | BAM M1–M6: Coulomb, viscous, Stribeck, load-dependent, directional, quadratic | |
| Load | J = m r² + J_spool + J_m; gravity −mgl·sin(θ + offset); 4 substeps per tick | `servo.json` setups |

## Data and split

- **Windows:** 6 s windows from the healthy runs only (`../data/runs.csv`). The 134414 run is used up to its burn (917 s).
- **Kinds:** trajectory (chirp, dual-sine, raise-lower at kp 8/15/30), drop (lift-drop releases), motor (laps, probes), chaos.
- **Validation:** two whole runs held out, `134414` (the 8.7 V repeat) and `112510` (7.4 V), both 1.52 kg at 50 mm.
- **Scoring:** simulation runs closed loop from the measured state. Motor and chaos windows are open loop in the duty and re-synced to the measurement every 1 s, as BAM's `reset_period`.
- **Optimizer:** CMA-ES, best of 3 seeds.

## Accuracy

BAM protocol: trajectory and drop windows, mean absolute angle error.

| Model | Train | Validation | BAM MX-64 | BAM MX-106 |
|---|---|---|---|---|
| M1 | 1.33° | 1.29° (0.0225 rad) | 0.027 rad | 0.019 rad |
| M2 | 1.33° | 1.29° (0.0226) | 0.025 | 0.019 |
| M3 | 1.15° | 0.88° (0.0154) | 0.021 | 0.0145 |
| M4 | 1.13° | 0.88° (0.0154) | 0.014 | 0.0105 |
| M5 | 1.08° | 0.83° (0.0145) | 0.0093 | 0.0095 |
| M6 | 1.08° | **0.81° (0.0141)** | 0.0093 | 0.0095 |

BAM's values are read off their Fig. 5 (±0.0005 rad).

M6 by setup and gain (validation, else training):

| Setup | kp 8 | kp 15 | kp 30 | Drop |
|---|---|---|---|---|
| 1.52 kg at 50 mm, held out | 0.56° | **0.47°** | 1.18° | 1.62° |
| 1.52 kg at 50 mm, train | 0.67° | 0.73° | 1.16° | 0.75° |
| 1.52 kg at 65 mm, train | 0.88° | 0.74° | 1.37° | 0.77° |
| 100 g at 125 mm, train | 1.25° | 1.85° | 2.63° | – |

Full model (`full-m6`), validation:

| Window | MAE |
|---|---|
| Trajectory | 0.72° |
| Drop | 1.54° |
| Chaos (1 s resync) | 3.3° |
| Motor (1 s resync) | 4.5° |

In training, chaos runs 2.0° (unloaded) to 4.1° (100 g), and motor 3.2° (1.52 kg at 50 mm) to 7.6° (1.52 kg at 65 mm).

Fitted `full-m6`:
- **Motor:** k 0.459 N·m/V, b 0.014 N·m·s/rad.
- **Inertia:** J_m 0.0137 kg·m², J_spool 0.0051 kg·m².
- **Friction:** Kc 0.053, Kv 0.393, Kcs 0.052, vs 0.22 rad/s, α 1.06, Km 0.145, Ke 0.301, Kmq 0.079, Keq 0.818.
- **Thermal:** aT 0.0051 /°C.
- **Controller:** kd 74.5.

## Verdict

**1. BAM-level accuracy: partly.**
- **Overall:** validation 0.81°, 1.5× BAM's best on the Dynamixels (0.53°).
- **Heavy pendulum at kp 8–15:** 0.47–0.56°, at BAM's level. 0.47° is 2.1 counts of this pot, where BAM's 0.53° is 6 counts of the MX-64's encoder.
- **The gap:**
  - kp 30: measured stick-slip steps of 3–5° that no friction model produces.
  - Light, fast load (100 g): only one light setup.
  - Drops: 11 torque-off windows.
- **M1 → M6 gain:** 1.6×, against 2.0–2.9× in BAM. Fewer setups means less to separate the friction terms.
- **Can't be closed with this data:** BAM used 9 mass/length combinations at 4 gains; this data has 3 setups (one light) at 3 gains, and unit 1 is destroyed.

**2. A model of everything simulable: the core yes; the edges are measured but not fitted.**

The fitted model reproduces the firmware path exactly: the output law, D on the delayed speed register, both slews, the goal ramp, mode-entry resets, and the quantized sensor with its blind spot. Torque scales with the measured supply and temperature. It holds 0.7° on trajectories and 3–4.5° over 1 s of open-loop motor mode. Not in the fitted model:

| Effect | What is known | Needed |
|---|---|---|
| Supply sag | 8.7 → 7.1–7.3 V at a full stall; 6.5–6.9 V at full output near standstill (bench supply) | Current sensing; a supply model per robot |
| Thermal state | Full-duty chaos: τ ≈ 20 min towards 80–90 °C reported. Hanging: 75 → 64 °C in 3.5 min. The sensor lags the winding | Heat input (current) |
| Overload clamp | Output held at 200 after 1–2 s of position stall short of the goal | The trigger rule: a stall ladder |
| Overheat cuts | Per-tick output → 0 on frames reading 100 °C; rare from 57 °C, dense at 74–75 °C | Rate vs temperature and current |
| Drive dropout | Once, at 72 °C: no current for > 70 s, no flag | Only operating limits (≤ 65 °C) |
| b vs Kv | Their sum (≈ 0.41 N·m·s/rad) is well identified, the split is not | More torque-off drops |
| Backlash | Not observable with the output-side pot | Motor-side or external encoder |
| Unit spread | ~10 % run to run; unit to unit unknown | A second unit |

## Use in a simulator

`sc_model.rollout` is the reference. Per joint, carry the slew state `u`,
the goal ramp and the 8-tick sensed-count history. Feed V (supply) and T
(temperature) from the robot's own model or randomize them, then apply
τ = f(T)(k u V − b ω) and the M6 friction in the physics step. For
MuJoCo, BAM's integration (`bam/mujoco.py`) shows how to apply extended
friction per step.
