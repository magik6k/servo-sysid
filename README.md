# Servo measurements

Bench characterisation of the serial-bus servos used in brobt, a small
purpose-built home robot that pushes, pulls and carries things on the
floor. The data feeds the actuator and sensor models of the RL simulator.
The interface plan is in `../docs/control-interface.md`.

| Servo | Status | Write-up | Charts |
|---|---|---|---|
| Waveshare SC15, unit 1 | Unloaded, arm, 100 g and 1.52 kg pendulum runs; full heavy batch next | [`sc15/README.md`](sc15/README.md) | [`sc15/rendered/`](sc15/rendered/README.md) |

## Layout

```
shared/                common to every servo
  firmware/            ESP32 (M5 Atom Lite) sketches; chaos/ is the test firmware
  tools/               recording and analysis (Python)
    servolab.py        shared helpers: run lookup, metadata, angle and derivative filtering
    chaos_log.py       records a chaos-firmware run into <servo>/data
    split_batch.py     splits a run into one CSV per block in <servo>/data/derived (not in git)
    hold_steps.py      position steps through the hold_position sketch
    chaos_analyze.py   slew, steady state, steps, laps, supply
    holds_analyze.py   gravity / gearbox efficiency / torque-per-unit fit from a holds run
  rig/                 OpenSCAD pendulum stand, arms, spool mount; STLs; vendor CAD
<servo>/               one directory per servo model (and unit)
  README.md            setup, register facts, every finding, run log, next run
  servo.json           constants: counts/rev, down count, sensor trust window, load setups
  data/                raw runs: <run>.csv.gz + <run>.meta.txt (preamble, plan, register dump, notes)
  render.py            draws the charts for this servo
  rendered/            README.md with the charts, charts/*.png (light and -dark)
```

## Workflow

1. Pick or add a block plan in `shared/firmware/chaos/chaos.ino`
   (`const auto &PLAN = …`) and flash it (see `shared/firmware/README.md`).
2. Record from the repo root:
   `python3 shared/tools/chaos_log.py --servo sc15 --seconds N --tag <load>-<what>`.
   This resets the board and writes `<servo>/data/chaos_<stamp>_<tag>.csv.gz`
   plus `.meta.txt`.
3. Add notes to the run's `.meta.txt` (`# NOTE …`) for anything the data
   alone doesn't say, such as setup changes or known bad blocks. Add a row
   to the servo README's run log.
4. Split into blocks if needed (`shared/tools/split_batch.py <run>.csv.gz`),
   analyse, and rerun `<servo>/render.py`.

Requires `pyserial`, `numpy`, `pandas`, `scipy` and `matplotlib`, plus
`arduino-cli` with the esp32 core ≥ 3.3.11.

## Conventions

- Telemetry is 250 Hz, one frame per 4 ms servo tick. Columns: `t_s,
  segment, mode, flags, command, command_time_ms, position, speed, load,
  voltage_v, temperature_c, error`, plus `block, kp` in v2 frames.
  `mode`: 0 motor, 1 position, 2 cooldown, 3 stall-guard trip, 4 released
  (torque off). `flags`: 0x01 segment start, 0x02 mode entry, 0x04 read
  failed.
- `load` is the PWM the servo actually applied, −1000..1000. It is the
  true actuator input; `command` is only what was asked.
- Angles are measured from hanging straight down (`servo.json`
  `down_count`). For the SC15, positive PWM drives the count down.
- Raw data is never edited. Corrections go into the `.meta.txt` as notes.
