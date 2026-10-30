# Firmware

Arduino sketches for the M5 Atom Lite (ESP32) driving SCS-protocol servos
on a one-wire bus (G33, 1 Mbaud, pull-up to 3.3 V). Flash with:

    arduino-cli compile --upload -p /dev/ttyUSB0 \
      --fqbn esp32:esp32:m5stack_atom:UploadSpeed=115200 shared/firmware/<sketch>

| Sketch | Use |
|---|---|
| `chaos/` | **The test firmware.** Runs a plan of blocks (chaos, laps, holds, chirp, dual-sine, raise-lower, lift-drop, swings, heavy laps, backlash, bang, torque probe, burn, burn_high, heat, …) and streams 26-byte binary frames at 250 Hz. Pick the plan with `const auto &PLAN = …`. Record with `shared/tools/chaos_log.py`. Servo-specific constants (`DOWN_COUNT`, `COUNTS_PER_DEG`, goal window) are at the top |
| `hold_position/` | Moves to a count and holds it; text telemetry at 20 Hz. Serial commands: `r` release, `h` hold here, `<count>` move. Used to park the horn while mounting arms, and by `shared/tools/hold_steps.py` |
| `bus_check/` | One-shot bus diagnosis: do broadcast and addressed writes land, and does the echo match |
| `reverse_probe/` | Probes for ways past the motor-mode PWM ramp (found the bounce) |
| `bounce_sweep/`, `reverse_test/` | Early reversal and bounce timing tests |
| `servo_test/`, `speed_sweep/`, `spin_cycle/`, `spin_sweep/` | First bring-up sketches: position sweeps and continuous rotation |

The early sketches print spec values (e.g. "0..1023 covers ~210°") that
measurement later corrected; see `sc15/README.md`.
