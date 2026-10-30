# servo-sysid

Bench characterisation and simulation models of hobby serial-bus servos,
for sim-to-real RL. Pendulum bench, 250 Hz telemetry of the PWM the
servo actually applies, BAM-style model fits.

| Servo | Data | Model | Write-up | Charts |
|---|---|---|---|---|
| Waveshare SC15 (Feetech SCS15-compatible), unit 1 | 23 runs, 3.2 h at 250 Hz: unloaded, arm, 100 g, 1.52 kg at 50 and 65 mm; 4.8–8.7 V; 21–75 °C. Unit destroyed in the last runs (stripped output gear), flagged | Firmware controller identified; BAM M1–M6 plus thermal; validation MAE 0.81° (BAM protocol) | [`sc15/README.md`](sc15/README.md), [`sc15/model/`](sc15/model/README.md) | [`sc15/rendered/`](sc15/rendered/README.md) |

## Layout

```
shared/                common to every servo
  firmware/            ESP32 (M5 Atom Lite) sketches; chaos/ is the test firmware
  tools/               recording, analysis, model (Python)
    servolab.py        run lookup, metadata, angle and derivative filtering
    chaos_log.py       records a chaos-firmware run into <servo>/data
    split_batch.py     one CSV per block into <servo>/data/derived (not in git)
    hold_steps.py      position steps through the hold_position sketch
    chaos_analyze.py   slew, steady state, steps, laps, supply
    holds_analyze.py   gravity / gearbox efficiency / torque-per-unit fit
    sc_model.py        actuator model: firmware controller, motor, friction, pendulum
    model_data.py      cuts runs into fitting windows
    fit_model.py       CMA-ES fit and held-out validation
  rig/                 OpenSCAD pendulum stand, arms, spool mount; STLs
<servo>/
  README.md            setup, register facts, findings, run log, next runs
  servo.json           counts/rev, down count, sensor trust window, load setups
  data/                raw runs: <run>.csv.gz + <run>.meta.txt; runs.csv = manifest with unit state
  model/               fitted parameters (JSON) and model card
  render.py            draws the charts
  rendered/            README.md with the charts (light and -dark PNGs)
```

## Data

All recordings are published as recorded; raw data is never edited.
Corrections are `# NOTE` lines in each run's `.meta.txt`. Per-run unit
state is in `<servo>/data/runs.csv`:

- `healthy`: use freely.
- `hot`: healthy hardware, but run at 63–75 °C reported, with overheat throttling.
- `faulty`: the unit was failing. `use_until_s` gives the last trusted time.

For the SC15, the output gear stripped during the hot runs at the end of
2026-10-30. The damage first shows 570 s into
`chaos_20261030_143007_hot-batch-1.52kg-at-50mm-8.7V`.

## Workflow

1. Pick or add a block plan in `shared/firmware/chaos/chaos.ino`
   (`const auto &PLAN = …`) and flash it (see `shared/firmware/README.md`).
2. Record from the repo root:
   `python3 shared/tools/chaos_log.py --servo sc15 --seconds N --tag <load>-<what>`.
   This resets the board and writes `<servo>/data/chaos_<stamp>_<tag>.csv.gz`
   plus `.meta.txt`.
3. Add `# NOTE` lines to the `.meta.txt`, and add a row to the run log and
   to `runs.csv`.
4. Analyse, fit (`shared/tools/fit_model.py`) and rerun `<servo>/render.py`.

`pip install -r requirements.txt`, plus `arduino-cli` with the esp32
core ≥ 3.3.11.

## Conventions

- Telemetry is 250 Hz, one frame per 4 ms servo tick. Columns: `t_s,
  segment, mode, flags, command, command_time_ms, position, speed, load,
  voltage_v, temperature_c, error`, plus `block, kp` in v2 frames.
  - `mode`: 0 motor, 1 position, 2 cooldown, 3 stall-guard trip, 4 released (torque off).
  - `flags`: 0x01 segment start, 0x02 mode entry, 0x04 read failed, 0x08 dead drive.
- `load` is the PWM the servo actually applied, −1000..1000. It is the
  true actuator input; `command` is only what was asked.
- Angles are measured from hanging straight down (`servo.json`
  `down_count`). For the SC15, positive PWM drives the count down.

## License

Code, data, CAD and documentation are dual-licensed under
[MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
