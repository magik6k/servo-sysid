#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_random.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 11)
#error "Install esp32 by Espressif Systems 3.3.11 or newer."
#endif

// Runs a plan of test blocks on the servo, with telemetry streamed as binary
// frames at the servo's own 250 Hz update rate. Record with shared/tools/chaos_log.py.
//   chaos        random segments across the whole range
//   laps         constant PWM levels in both directions, several turns each
//   holds        position-mode steps across the sensed window, up then down
//   chirp        60 deg sine, frequency rising 0.2 -> 1.5 Hz (BAM-style)
//   dual_sine    slow 70 deg swing with a fast 10 deg wiggle (BAM-style)
//   raise_lower  slow ramps to +/-100 deg and back (BAM-style)
//   lift_drop    lift to an angle, then torque off and let it fall (BAM-style)
//   swings       position-mode moves to growing angles, alternating sides
//   heavy_laps   laps for a heavy pendulum: from 35 %, ramped stops
//   check_laps   heavy laps at 50 % only, one level each way
//   regen_bursts three 5 s bursts of 80 % laps: repeated overhauling falls
//   climb        one flat-out position move from down to +100 deg, slow return
//   bang         flat out up and down near horizontal, reversing at full speed
//   chirp_soft, dual_sine_soft  smaller, slower versions for a heavy pendulum
//   backlash     1-8 deg steps out and back, hanging down and at 45 deg
//   burn         full-duty chaos with a probe every minute, until the servo's
//                own overheat protection trips; then torque off to cool
//   burn_high    burn that brakes flat out while the pendulum falls through
//                the lower half, with 6 s of normal chaos every 2 minutes
//   heat         a narrow bang near horizontal until the servo is back up to
//                near its limit; put before each block of a plan to run it hot
// Each block can set the position-loop P gain. After the plan the gain is
// restored and the pendulum is parked hanging down.

constexpr int DATA_PIN = 33;  // G33; use 26 for Grove yellow.
constexpr uint32_t SERVO_BAUD = 1000000;
constexpr uint8_t SERVO_ID = 1;

constexpr uint32_t SEED = 0;          // 0 = new seed each boot (logged).
constexpr uint32_t SAMPLE_US = 4000;  // Servo updates every 4 ms.

constexpr int16_t MAX_DUTY = 1000;
constexpr uint32_t MIN_HOLD_MS = 20;
constexpr uint32_t MAX_HOLD_MS = 2000;

constexpr uint8_t HOT_C = 65;   // Torque off at this temperature...
constexpr uint8_t COOL_C = 50;  // ...until it is back down to this.
constexpr uint8_t TEMP_READS = 3;  // Consecutive readings: single-frame glitches reach +15 C.

// Plans with heat blocks run near the servo's limit instead. Under load it
// throttles itself from ~74 C reported (single 100 C readings with the
// overheat bit, each cutting the output to 0), and at ~72 C the motor
// drive once cut out silently for over a minute, so they stay below that.
// Each heat block brings the temperature back up to HEAT_TARGET_C unless
// it is still at HEAT_FLOOR_C or more.
constexpr uint8_t HOT_RUN_HOT_C = 69;
constexpr uint8_t HOT_RUN_COOL_C = 66;
constexpr uint8_t HEAT_FLOOR_C = 63;
constexpr uint8_t HEAT_TARGET_C = 66;

// A stall guard trip at DEAD_DRIVE_MIN_LOAD or more with the supply no
// lower than at idle means no current is flowing: the drive is dead (as
// at 72 C), not stalled. Torque goes off until the cooldown temperature.
constexpr int16_t DEAD_DRIVE_MIN_LOAD = 500;

// Torque off if the motor is driven hard but the shaft stops inside the
// sensed range: stalled against gravity or something in the way. That
// segment ends there and the next one turns torque back on. 0 disables.
// Long enough that a heavy pendulum can sit at a motor-mode equilibrium
// (static torque data) for most of a chaos segment.
constexpr uint32_t STALL_GUARD_MS = 1500;
constexpr int16_t STALL_MIN_LOAD = 150;           // Motor mode.
constexpr int16_t POSITION_STALL_MIN_LOAD = 800;  // Position mode: well above P-term holds.

// Pendulum geometry: the arm hangs straight down at DOWN_COUNT.
constexpr uint16_t DOWN_COUNT = 513;
constexpr float COUNTS_PER_DEG = 1610.0f / 360.0f;
constexpr int16_t GOAL_MIN = 40;   // Stay inside the trusted sensor window.
constexpr int16_t GOAL_MAX = 985;
constexpr uint8_t DEFAULT_KP = 15;  // Factory position-loop P gain (0x15).

enum Block : uint8_t {
  B_CHAOS, B_LAPS, B_HOLDS, B_CHIRP, B_DUAL_SINE, B_RAISE_LOWER, B_LIFT_DROP,
  B_SWINGS, B_HEAVY_LAPS, B_CHECK_LAPS, B_REGEN_BURSTS, B_CLIMB, B_BANG,
  B_CHIRP_SOFT, B_DUAL_SINE_SOFT, B_BACKLASH, B_PROBE, B_BURN, B_BURN_HIGH, B_HEAT, B_DONE
};
const char *const BLOCK_NAMES[] = {
  "chaos", "laps", "holds", "chirp", "dual_sine", "raise_lower", "lift_drop",
  "swings", "heavy_laps", "check_laps", "regen_bursts", "climb", "bang",
  "chirp_soft", "dual_sine_soft", "backlash", "probe", "burn", "burn_high", "heat", "done"
};

constexpr uint32_t CHIRP_MS = 20000;
constexpr uint32_t DUAL_SINE_MS = 20000;
constexpr uint32_t RAISE_LOWER_MS = 34000;

struct BlockSpec {
  Block block;
  uint8_t kp;
  uint32_t durationMs;  // chaos and streamed trajectories; scripts set their own
};

// Everything that can be done with one pendulum mass.
const BlockSpec BATCH[] = {
  {B_CHAOS, 15, 300000},
  {B_LAPS, 15, 0},
  {B_HOLDS, 15, 0},
  {B_CHIRP, 8, CHIRP_MS},  {B_DUAL_SINE, 8, DUAL_SINE_MS},  {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_CHIRP, 15, CHIRP_MS}, {B_DUAL_SINE, 15, DUAL_SINE_MS}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_CHIRP, 30, CHIRP_MS}, {B_DUAL_SINE, 30, DUAL_SINE_MS}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
  {B_LIFT_DROP, 15, 0},
};

// Just the streamed BAM-style trajectories, at each gain.
const BlockSpec TRAJECTORIES[] = {
  {B_CHIRP, 8, CHIRP_MS},  {B_DUAL_SINE, 8, DUAL_SINE_MS},  {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_CHIRP, 15, CHIRP_MS}, {B_DUAL_SINE, 15, DUAL_SINE_MS}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_CHIRP, 30, CHIRP_MS}, {B_DUAL_SINE, 30, DUAL_SINE_MS}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
};

// First run with a heavy pendulum: swings out to the sensor limits, then
// whole turns.
const BlockSpec HEAVY_FIRST[] = {
  {B_SWINGS, 15, 0},
  {B_HEAVY_LAPS, 15, 0},
};

// Regen check after adding a supply clamp: the fast swings and 50 % laps
// that hit the 9.9 V reading ceiling before.
const BlockSpec REGEN_CHECK[] = {
  {B_SWINGS, 15, 0},
  {B_CHECK_LAPS, 15, 0},
};

// Heat soak of the supply clamp: back-to-back regen bursts.
const BlockSpec REGEN_SOAK[] = {
  {B_REGEN_BURSTS, 15, 0},
};

// One full-power climb, to check the supply's current limit on a scope.
const BlockSpec FULL_CLIMB[] = {
  {B_CLIMB, 15, 0},
};

// Worst case for the supply and gearbox: full-speed reversals at the
// heaviest gravity load.
const BlockSpec BANG_TEST[] = {
  {B_BANG, 30, 0},  // kp 30: the output saturates until ~7 deg from the goal
};

// Everything, with a heavy pendulum (~1 N·m). Slow and gentle first,
// chaos last because it heats the servo most. ~14.5 min plus any cooldown.
const BlockSpec HEAVY_BATCH[] = {
  {B_HOLDS, 15, 0},
  {B_BACKLASH, 15, 0}, {B_BACKLASH, 8, 0},
  {B_CHIRP_SOFT, 8, CHIRP_MS},  {B_DUAL_SINE_SOFT, 8, DUAL_SINE_MS},  {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 15, CHIRP_MS}, {B_DUAL_SINE_SOFT, 15, DUAL_SINE_MS}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 30, CHIRP_MS}, {B_DUAL_SINE_SOFT, 30, DUAL_SINE_MS}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
  {B_LIFT_DROP, 15, 0},
  {B_HEAVY_LAPS, 15, 0},
  {B_BANG, 30, 0},
  {B_CHAOS, 15, 300000},
};

// HEAVY_BATCH with a torque probe before, between and after the groups,
// so every block has a torque-vs-temperature reference. ~15.5 min.
const BlockSpec HEAVY_BATCH_PROBED[] = {
  {B_PROBE, 15, 0},
  {B_HOLDS, 15, 0},
  {B_BACKLASH, 15, 0}, {B_BACKLASH, 8, 0},
  {B_PROBE, 15, 0},
  {B_CHIRP_SOFT, 8, CHIRP_MS},  {B_DUAL_SINE_SOFT, 8, DUAL_SINE_MS},  {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 15, CHIRP_MS}, {B_DUAL_SINE_SOFT, 15, DUAL_SINE_MS}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 30, CHIRP_MS}, {B_DUAL_SINE_SOFT, 30, DUAL_SINE_MS}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
  {B_PROBE, 15, 0},
  {B_LIFT_DROP, 15, 0},
  {B_HEAVY_LAPS, 15, 0},
  {B_BANG, 30, 0},
  {B_PROBE, 15, 0},
  {B_CHAOS, 15, 300000},
  {B_PROBE, 15, 0},
};

// HEAVY_BATCH_PROBED, then a burn to the servo's overheat trip and a
// torque-off cooldown that lasts until the recording stops.
// WARNING: the burn, burn_high and hot plans below destroyed SC15 unit 1
// (stripped output gear with 1.52 kg at 70-75 C). Keep heavy loads cooler.
const BlockSpec HEAVY_BATCH_BURN[] = {
  {B_PROBE, 15, 0},
  {B_HOLDS, 15, 0},
  {B_BACKLASH, 15, 0}, {B_BACKLASH, 8, 0},
  {B_PROBE, 15, 0},
  {B_CHIRP_SOFT, 8, CHIRP_MS},  {B_DUAL_SINE_SOFT, 8, DUAL_SINE_MS},  {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 15, CHIRP_MS}, {B_DUAL_SINE_SOFT, 15, DUAL_SINE_MS}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_CHIRP_SOFT, 30, CHIRP_MS}, {B_DUAL_SINE_SOFT, 30, DUAL_SINE_MS}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
  {B_PROBE, 15, 0},
  {B_LIFT_DROP, 15, 0},
  {B_HEAVY_LAPS, 15, 0},
  {B_BANG, 30, 0},
  {B_PROBE, 15, 0},
  {B_CHAOS, 15, 300000},
  {B_PROBE, 15, 0},
  {B_BURN, 15, 0},
};

// burn_high straight away, for a servo that is already hot (a probe
// block first would hit the HOT_C cutoff; the burn has its own probes).
const BlockSpec BURN_HIGH_ONLY[] = {
  {B_BURN_HIGH, 15, 0},
};

// HEAVY_BATCH_PROBED as close to overheat as it runs: a heat block before
// each block, and the higher cutoff.
const BlockSpec HOT_BATCH[] = {
  {B_HEAT, 30, 0}, {B_PROBE, 15, 0},
  {B_HEAT, 30, 0}, {B_HOLDS, 15, 0},
  {B_HEAT, 30, 0}, {B_BACKLASH, 15, 0}, {B_HEAT, 30, 0}, {B_BACKLASH, 8, 0},
  {B_HEAT, 30, 0}, {B_PROBE, 15, 0},
  {B_HEAT, 30, 0}, {B_CHIRP_SOFT, 8, CHIRP_MS},
  {B_HEAT, 30, 0}, {B_DUAL_SINE_SOFT, 8, DUAL_SINE_MS},
  {B_HEAT, 30, 0}, {B_RAISE_LOWER, 8, RAISE_LOWER_MS},
  {B_HEAT, 30, 0}, {B_CHIRP_SOFT, 15, CHIRP_MS},
  {B_HEAT, 30, 0}, {B_DUAL_SINE_SOFT, 15, DUAL_SINE_MS},
  {B_HEAT, 30, 0}, {B_RAISE_LOWER, 15, RAISE_LOWER_MS},
  {B_HEAT, 30, 0}, {B_CHIRP_SOFT, 30, CHIRP_MS},
  {B_HEAT, 30, 0}, {B_DUAL_SINE_SOFT, 30, DUAL_SINE_MS},
  {B_HEAT, 30, 0}, {B_RAISE_LOWER, 30, RAISE_LOWER_MS},
  {B_HEAT, 30, 0}, {B_PROBE, 15, 0},
  {B_HEAT, 30, 0}, {B_LIFT_DROP, 15, 0},
  {B_HEAT, 30, 0}, {B_HEAVY_LAPS, 15, 0},
  {B_HEAT, 30, 0}, {B_BANG, 30, 0},
  {B_HEAT, 30, 0}, {B_PROBE, 15, 0},
  {B_HEAT, 30, 0}, {B_CHAOS, 15, 300000},
  {B_HEAT, 30, 0}, {B_PROBE, 15, 0},
};

// What to run, in order.
const auto &PLAN = HEAVY_BATCH_PROBED;
constexpr size_t PLAN_COUNT = sizeof(PLAN) / sizeof(PLAN[0]);
constexpr size_t START_BLOCK = 0;  // To resume a plan after an interruption (logged).

// HOLDS: steps every HOLD_STEP_DEG, sweeping up then back down.
constexpr int HOLD_MAX_DEG = 100;
constexpr int HOLD_STEP_DEG = 10;
constexpr uint32_t HOLD_MS = 2500;
constexpr uint16_t HOLD_MOVE_MS = 600;

// Streamed trajectories: a new goal every sample, reached within one tick.
constexpr uint16_t TRAJ_TIME_MS = SAMPLE_US / 1000;

// LIFT_DROP: lift to each angle, then release for DROP_RELEASE_MS.
const int16_t DROP_DEG[] = {30, 60, 90, 105, -30, -60, -90, -105};
constexpr size_t DROP_COUNT = sizeof(DROP_DEG) / sizeof(DROP_DEG[0]);
constexpr uint32_t DROP_RELEASE_MS = 3000;

// PROBE: from hanging still, a fixed motor-mode push each way. Against a
// heavy pendulum it stalls where torque = gravity, so the stall angle
// tracks torque per unit as the motor heats; the stall guard ends the
// push. 25 % with 1.52 kg at 50 mm: roughly 55 deg cold, 40 deg warm.
constexpr int16_t PROBE_DUTY = 250;
constexpr uint32_t PROBE_PUSH_MS = 4000;
constexpr uint32_t PROBE_SEGMENTS = 5;  // settle, push +, settle, push -, settle

// BURN: random-length full-duty motor segments, either direction, with
// the HOT_C cutoff off. A probe every BURN_PROBE_EVERY_MS tracks torque
// as it heats. It ends when the servo reports overheat (limit 0x0D = 80 C,
// unload mask 0x13 = 0x24), or at BURN_MAX_C if it never does. The command
// is kept for BURN_WATCH_MS after that, to see what the servo does by
// itself, then torque goes off and the pendulum cools hanging.
constexpr uint8_t ERROR_OVERHEAT = 0x04;
constexpr uint8_t BURN_MAX_C = 85;
constexpr uint32_t BURN_PROBE_EVERY_MS = 60000;
constexpr uint32_t BURN_WATCH_MS = 3000;

// BURN_HIGH: as BURN, but while the pendulum falls through the lower
// half it drives flat out against the motion, so the energy of the fall
// goes into the motor (and back to the supply) instead of into speed.
// Each switch bounces the mode, so the output restarts from 0 instead of
// slewing through the old duty. Every BURN_CHAOS_EVERY_MS it runs
// BURN_CHAOS_MS of normal chaos segments.
constexpr float BRAKE_DEG = 90;
constexpr int16_t BRAKE_COUNTS = BRAKE_DEG * COUNTS_PER_DEG;
constexpr int16_t BRAKE_MIN_SPEED = 200;  // counts/s; slower swings run the segment's duty
constexpr int16_t BRAKE_MAX_STEP = 30;    // counts/sample; larger is a gap sweep, not motion
constexpr uint32_t BURN_CHAOS_EVERY_MS = 120000;
constexpr uint32_t BURN_CHAOS_MS = 6000;

// BANG: from rest at BANG_LO_DEG, flat out up and down. Each goal lies
// past its flip point so the position loop stays saturated until the
// flip (the plan runs it at kp 30). 45 deg above horizontal is in the
// blind spot, so the band sits around horizontal instead.
constexpr float BANG_LO_DEG = 58;     // Flip up when falling past this...
constexpr float BANG_HI_DEG = 97;     // ...and down when rising past this.
constexpr float BANG_LO_GOAL = 45;
constexpr float BANG_HI_GOAL = 105;
constexpr uint8_t BANG_STROKES = 6;   // 3 up, 3 down
constexpr uint32_t BANG_TIMEOUT_MS = 6000;

// HEAT: the most current gravity allows without a stop is flat out at low
// speed with the arm extended, so a narrow bang just below horizontal,
// with no stroke limit. Full-speed laps mostly fight back-EMF, and holds
// trip the overload clamp. Sides alternate between heat blocks.
constexpr float HEAT_LO_DEG = 75;
constexpr float HEAT_HI_DEG = 95;
constexpr float HEAT_LO_GOAL = 60;
constexpr float HEAT_HI_GOAL = 105;
constexpr uint32_t HEAT_SEGMENT_MS = 30000;

// BACKLASH: small steps out and back around each centre. Hanging down
// nothing preloads the gears; at 45 deg gravity holds them against one
// flank, so the difference is the play.
const int16_t BACKLASH_CENTRES_DEG[] = {0, 45};
const float BACKLASH_STEPS_DEG[] = {1, 2, 4, 8};
constexpr size_t BACKLASH_STEP_COUNT = sizeof(BACKLASH_STEPS_DEG) / sizeof(BACKLASH_STEPS_DEG[0]);
constexpr size_t BACKLASH_PER_CENTRE = 1 + 4 * BACKLASH_STEP_COUNT;  // settle, then +a 0 -a 0
constexpr size_t BACKLASH_COUNT = BACKLASH_PER_CENTRE * sizeof(BACKLASH_CENTRES_DEG) / sizeof(BACKLASH_CENTRES_DEG[0]);
constexpr uint16_t BACKLASH_MOVE_MS = 300;
constexpr uint32_t BACKLASH_HOLD_MS = 1500;

// CLIMB: settle hanging down, flat out to +100 deg (the P term saturates
// at full output), hold, then a slow return that doesn't regen.
struct Move {
  int16_t deg;
  uint16_t moveMs;
  uint32_t holdMs;
};
const Move CLIMB[] = {{0, 1000, 2000}, {100, 1, 2500}, {0, 3000, 4000}};
constexpr size_t CLIMB_COUNT = sizeof(CLIMB) / sizeof(CLIMB[0]);

// SWINGS: move to each angle in SWING_MOVE_MS, then hold.
const int16_t SWING_DEG[] = {30, -30, 60, -60, 90, -90, 105, -105, 0};
constexpr size_t SWING_COUNT = sizeof(SWING_DEG) / sizeof(SWING_DEG[0]);
constexpr uint16_t SWING_MOVE_MS = 2000;
constexpr uint32_t SWING_HOLD_MS = 1000;

struct Lap {
  int16_t duty;
  uint32_t holdMs;
};

// Each level runs long enough for 3+ turns (~1610 counts/turn at
// ~2.12 counts/s per PWM unit), with a bounce-stop between.
const Lap LAPS[] = {
  {0, 2000},
  {150, 18000}, {0, 1500}, {-150, 18000}, {0, 1500},
  {250, 11000}, {0, 1500}, {-250, 11000}, {0, 1500},
  {400, 8000},  {0, 1500}, {-400, 8000},  {0, 1500},
  {600, 6000},  {0, 1500}, {-600, 6000},  {0, 1500},
  {800, 6000},  {0, 1500}, {-800, 6000},  {0, 1500},
  {1000, 6000}, {0, 1500}, {-1000, 6000},
};
constexpr size_t LAP_COUNT = sizeof(LAPS) / sizeof(LAPS[0]);

// Heavy pendulum: below ~35 % it may not get past horizontal. No bounce:
// stops ramp down at the firmware slew instead of snapping the load to a
// halt. Capped at 80 % until regen with this load is known.
const Lap HEAVY_LAPS[] = {
  {0, 1500},
  {350, 7000}, {0, 2000}, {-350, 7000}, {0, 2000},
  {500, 5000}, {0, 2000}, {-500, 5000}, {0, 2000},
  {650, 4000}, {0, 2000}, {-650, 4000}, {0, 2000},
  {800, 4000}, {0, 2000}, {-800, 4000}, {0, 2000},
};
constexpr size_t HEAVY_LAP_COUNT = sizeof(HEAVY_LAPS) / sizeof(HEAVY_LAPS[0]);

const Lap CHECK_LAPS[] = {
  {0, 1500}, {500, 5000}, {0, 2000}, {-500, 5000}, {0, 2000},
};
constexpr size_t CHECK_LAP_COUNT = sizeof(CHECK_LAPS) / sizeof(CHECK_LAPS[0]);

// ~1 turn/s at 80 %, so each burst is ~5 falls at ~530 deg/s.
const Lap REGEN_BURSTS[] = {
  {0, 1500},
  {800, 5000}, {0, 3000}, {-800, 5000}, {0, 3000}, {800, 5000}, {0, 3000},
};
constexpr size_t REGEN_BURST_COUNT = sizeof(REGEN_BURSTS) / sizeof(REGEN_BURSTS[0]);

// A position-mode segment started outside the sensed window jerks at
// full power, so after motor-mode segments the shaft first creeps back in
// at this duty (for at most CREEP_MAX_MS). After the plan it parks down.
constexpr int16_t CREEP_DUTY = 200;
constexpr uint32_t CREEP_MAX_MS = 3000;
constexpr uint16_t PARK_MOVE_MS = 2500;

const uint8_t MOTOR_MODE[] = {0x00, 0x00, 0x00, 0x00};     // Angle limits 0/0.
const uint8_t POSITION_MODE[] = {0x00, 0x00, 0x03, 0xFF};  // Angle limits 0/1023.

enum Mode : uint8_t { MOTOR = 0, POSITION = 1, COOLDOWN = 2, FAULT = 3, RELEASED = 4 };

constexpr uint8_t FLAG_SEGMENT_START = 0x01;  // First frame of a segment.
constexpr uint8_t FLAG_MODE_ENTRY = 0x02;     // Mode limits written: ramp reset.
constexpr uint8_t FLAG_READ_FAILED = 0x04;    // No telemetry in this frame.
constexpr uint8_t FLAG_DEAD_DRIVE = 0x08;     // Stalled at high output with no supply sag.

// Little-endian, 26 bytes. Checksum = sum of bytes after sync, mod 256.
struct __attribute__((packed)) Frame {
  uint8_t sync[2];       // 0xA5 0x5A
  uint32_t timeUs;       // micros() when the read was sent
  uint16_t segment;
  uint8_t mode;          // Mode
  uint8_t flags;
  int16_t command;       // Motor: duty -1000..1000. Position: goal 0..1023.
  uint16_t commandTime;  // Position: goal time in ms. Motor: 0.
  uint16_t position;     // 0..1023 over ~227 deg; pinned in the blind spot
  int16_t speed;         // counts/s; garbage in the blind spot
  int16_t load;          // PWM output actually applied, -1000..1000
  uint8_t voltage;       // 0.1 V
  uint8_t temperature;   // deg C
  uint8_t error;         // Reply ERROR byte
  uint8_t block;         // Block
  uint8_t kp;            // Position-loop P gain in effect
  uint8_t checksum;
};
static_assert(sizeof(Frame) == 26, "Frame layout is shared with chaos_log.py");

struct Segment {
  Mode mode;
  bool bounce;           // Motor: bounce through position mode first.
  bool streaming;        // Position: goal follows the block's trajectory.
  bool release;          // Torque off for the whole segment.
  bool brake;            // Motor: BURN_HIGH braking while falling.
  int16_t command;
  uint16_t commandTime;
  uint32_t holdMs;
};

HardwareSerial ServoBus(1);

uint32_t rngState = 1;
Segment segment{};
uint16_t segmentIndex = 0;
uint32_t segmentStartMs = 0;
bool segmentModeEntry = false;
Mode servoMode = COOLDOWN;  // Unknown until the first segment sets it.
bool released = false;
bool cooling = false;
bool stalled = false;  // Stall guard tripped in this segment.
uint16_t stallPosition = 0;
uint16_t lastPosition = DOWN_COUNT;  // Latest good reading.
Segment pending{};  // Next planned segment, held back while creeping in.
bool havePending = false;
uint32_t creepSinceMs = 0;
bool bangUp = true;
uint8_t bangStrokes = 0;
uint32_t stallSinceMs = 0;
uint32_t nextSampleUs = 0;
bool burnTripped = false;
uint32_t burnProbeStep = 0;  // Next probe segment; 0 = between probes.
uint32_t burnProbeAtMs = 0;
uint32_t burnChaosAtMs = 0;
uint32_t burnChaosEndMs = 0;
int16_t lastSpeed = 0;     // Latest good reading, counts/s.
int16_t positionStep = 0;  // Counts since the reading before.
bool braking = false;
int16_t appliedDuty = 0;   // Brake segments: what the PWM register holds.
bool rampReset = false;    // Bounced after the last frame.
uint8_t recentTemps[3] = {};
uint8_t tempCount = 0;
uint8_t steadyTemp = 0;    // Median of the last 3 readings.
bool hotPlan = false;      // The plan has heat blocks.
int heatSide = -1;         // +1 or -1: which side the current heat block bangs on.

size_t blockIndex = START_BLOCK;
uint32_t blockSegment = 0;
uint32_t blockStartMs = 0;
uint8_t currentKp = 0;  // Unknown until the first block sets it.

uint32_t rng() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

int32_t uniform(int32_t lo, int32_t hi) {
  return lo + static_cast<int32_t>(rng() % static_cast<uint32_t>(hi - lo + 1));
}

uint32_t logUniform(uint32_t lo, uint32_t hi) {
  const float u = rng() / 4294967296.0f;
  return static_cast<uint32_t>(lo * powf(static_cast<float>(hi) / lo, u));
}

// Sends one instruction packet and copies it into packet.
// Returns the packet length.
uint8_t sendPacket(uint8_t id, uint8_t instruction,
                   const uint8_t *params, uint8_t count, uint8_t *packet) {
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = count + 2;  // Packet length
  packet[4] = instruction;
  memcpy(packet + 5, params, count);

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 5 + count; ++i) {
    checksum += packet[i];
  }
  packet[5 + count] = static_cast<uint8_t>(~checksum);

  // Drop stale echo from earlier writes so replies line up.
  while (ServoBus.available()) {
    ServoBus.read();
  }

  ServoBus.write(packet, count + 6);
  ServoBus.flush(true);  // Wait for transmission to finish.
  return count + 6;
}

// Broadcast WRITE of 1..6 consecutive register bytes; no reply is requested.
void servoWrite(uint8_t address, const uint8_t *data, uint8_t count) {
  if (count == 0 || count > 6) return;

  uint8_t params[7] = {address};
  memcpy(params + 1, data, count);

  uint8_t packet[13];
  sendPacket(0xFE, 0x03, params, count + 1, packet);
}

// READ of 1..64 bytes from one servo. The shared data line also hands back
// our own transmission, so the local echo is skipped before parsing.
// Returns the reply's ERROR status byte, or -1 if no valid reply arrived.
int servoRead(uint8_t id, uint8_t address, uint8_t count, uint8_t *out) {
  if (count == 0 || count > 64) return -1;

  const uint8_t params[] = {address, count};
  uint8_t packet[8];
  const uint8_t sent = sendPacket(id, 0x02, params, sizeof(params), packet);

  // Reply: FF FF id len error data[count] checksum, with len = count + 2.
  uint8_t buf[160];
  size_t n = 0;
  const size_t expected = sent + count + 6;
  const uint32_t startUs = micros();
  while (n < expected && micros() - startUs < 3000) {
    if (ServoBus.available()) {
      buf[n++] = ServoBus.read();
    }
  }

  size_t i = 0;
  if (n >= sent && memcmp(buf, packet, sent) == 0) {
    i = sent;
  }

  for (; i + count + 6 <= n; ++i) {
    if (buf[i] != 0xFF || buf[i + 1] != 0xFF ||
        buf[i + 2] != id || buf[i + 3] != count + 2) {
      continue;
    }

    uint8_t checksum = 0;
    for (size_t j = i + 2; j < i + 5 + count; ++j) {
      checksum += buf[j];
    }
    if (static_cast<uint8_t>(~checksum) != buf[i + 5 + count]) continue;

    memcpy(out, buf + i + 5, count);
    return buf[i + 4];
  }
  return -1;
}

void dumpRegisters() {
  for (uint8_t base = 0x00; base < 0x50; base += 16) {
    uint8_t data[16];
    Serial.printf("# reg %02X:", base);

    if (servoRead(SERVO_ID, base, sizeof(data), data) < 0) {
      Serial.println(" (no reply)");
      continue;
    }
    for (uint8_t b : data) {
      Serial.printf(" %02X", b);
    }
    Serial.println();
  }
}

// In motor mode 0x2C holds PWM duty; bit 10 selects reverse.
void setDuty(int16_t value) {
  const uint16_t raw = value < 0
    ? static_cast<uint16_t>(-value) | 0x0400
    : static_cast<uint16_t>(value);

  // SC-series values are big-endian: high byte first.
  const uint8_t data[] = {
    static_cast<uint8_t>(raw >> 8),
    static_cast<uint8_t>(raw)
  };
  servoWrite(0x2C, data, sizeof(data));
}

void writeGoal(int16_t goal, uint16_t timeMs) {
  const uint8_t move[] = {
    static_cast<uint8_t>(goal >> 8),
    static_cast<uint8_t>(goal),
    static_cast<uint8_t>(timeMs >> 8),
    static_cast<uint8_t>(timeMs),
    0x00, 0x00  // Speed 0: use the time.
  };
  servoWrite(0x2A, move, sizeof(move));
}

void setTorque(bool on) {
  const uint8_t value = on ? 1 : 0;
  servoWrite(0x28, &value, 1);
}

// Position-loop P gain lives in the EEPROM area; with the lock flag on it
// applies at once but is not saved, so a power cycle restores the default.
void setKp(uint8_t kp) {
  if (kp == currentKp) return;
  servoWrite(0x15, &kp, 1);
  currentKp = kp;
}

int signMagnitude(uint16_t value, uint8_t signBit) {
  const int magnitude = value & ((1u << signBit) - 1);
  return value & (1u << signBit) ? -magnitude : magnitude;
}

int16_t goalCount(float deg) {
  const int32_t goal = DOWN_COUNT + lroundf(deg * COUNTS_PER_DEG);
  return static_cast<int16_t>(constrain(goal, GOAL_MIN, GOAL_MAX));
}

// Pendulum angle from hanging down, in degrees, t seconds into a streamed
// trajectory block.
// Logged in the preamble so analysis knows the exact shapes.
const char TRAJECTORY_INFO[] =
  "chirp 60deg 0.2->1.5Hz; chirp_soft 45deg 0.2->1.0Hz; "
  "dual_sine 70deg@0.15Hz+10deg@2.5Hz; dual_sine_soft 60deg@0.15Hz+8deg@2.0Hz; "
  "raise_lower 12.5deg/s to +-100 with 1 s pauses";

float chirpDeg(float amplitude, float f0, float f1, float t) {
  const float duration = CHIRP_MS / 1000.0f;
  return amplitude * sinf(TWO_PI * (f0 * t + (f1 - f0) * t * t / (2 * duration)));
}

float trajectoryDeg(Block kind, float t) {
  switch (kind) {
    case B_CHIRP:
      return chirpDeg(60.0f, 0.2f, 1.5f, t);
    case B_CHIRP_SOFT:  // Peak ~280 deg/s: about the heavy pendulum's top lifting speed.
      return chirpDeg(45.0f, 0.2f, 1.0f, t);
    case B_DUAL_SINE:
      return 70.0f * sinf(TWO_PI * 0.15f * t) + 10.0f * sinf(TWO_PI * 2.5f * t);
    case B_DUAL_SINE_SOFT:
      return 60.0f * sinf(TWO_PI * 0.15f * t) + 8.0f * sinf(TWO_PI * 2.0f * t);
    case B_RAISE_LOWER: {
      // 12.5 deg/s up to +100, pause, back, the same on the other side.
      static const float knots[][2] = {
        {0, 0}, {8, 100}, {9, 100}, {17, 0}, {25, -100}, {26, -100}, {34, 0}
      };
      for (size_t k = 1; k < sizeof(knots) / sizeof(knots[0]); ++k) {
        if (t <= knots[k][0]) {
          const float a = (t - knots[k - 1][0]) / (knots[k][0] - knots[k - 1][0]);
          return knots[k - 1][1] + a * (knots[k][1] - knots[k - 1][1]);
        }
      }
      return 0;
    }
    default:
      return 0;
  }
}

Segment moveSegment(float deg, uint16_t moveMs, uint32_t holdMs) {
  Segment s{};
  s.mode = POSITION;
  s.command = goalCount(deg);
  s.commandTime = moveMs;
  s.holdMs = holdMs;
  return s;
}

Segment randomSegment() {
  Segment s{};

  if (uniform(0, 99) < 75) {
    s.mode = MOTOR;
    s.bounce = rng() & 1;
    s.holdMs = logUniform(MIN_HOLD_MS, MAX_HOLD_MS);

    const int32_t pick = uniform(0, 9);
    const int16_t sign = rng() & 1 ? 1 : -1;
    if (pick == 0) {
      s.command = 0;
    } else if (pick == 1) {
      s.command = sign * MAX_DUTY;
    } else if (pick == 2) {
      s.command = sign * uniform(0, 300);  // Dense near the deadband.
    } else {
      s.command = uniform(-MAX_DUTY, MAX_DUTY);
    }
  } else {
    s.mode = POSITION;
    s.command = uniform(GOAL_MIN, GOAL_MAX);
    s.commandTime = rng() & 1 ? 1 : uniform(50, 1000);  // 1 = flat out
    s.holdMs = uniform(300, 1500);
  }
  return s;
}

Segment probeSegment(uint32_t i) {
  if (i % 2 == 0) return i == 0 ? moveSegment(0, 1000, 2500) : moveSegment(0, 2000, 3000);
  Segment s{};
  s.mode = MOTOR;
  s.command = i == 1 ? PROBE_DUTY : -PROBE_DUTY;
  s.holdMs = PROBE_PUSH_MS;
  return s;
}

Segment fullDutySegment(bool brake) {
  Segment s{};
  s.mode = MOTOR;
  s.bounce = rng() & 1;
  s.brake = brake;
  s.command = rng() & 1 ? MAX_DUTY : -MAX_DUTY;
  s.holdMs = logUniform(MIN_HOLD_MS, MAX_HOLD_MS);
  return s;
}

Segment burnSegment(uint32_t i, bool high) {
  Segment s{};
  if (burnTripped) {  // Torque off until the recording stops.
    s.mode = POSITION;
    s.release = true;
    s.holdMs = UINT32_MAX;
    return s;
  }
  if (i == 0) {
    burnProbeAtMs = millis() + BURN_PROBE_EVERY_MS;  // BURN follows a probe.
    burnChaosAtMs = millis() + BURN_CHAOS_EVERY_MS;
  }
  if (high && millis() < burnChaosEndMs) return randomSegment();
  if (burnProbeStep > 0 || millis() >= burnProbeAtMs) {
    s = probeSegment(burnProbeStep++);
    if (burnProbeStep == PROBE_SEGMENTS) {
      burnProbeStep = 0;
      burnProbeAtMs = millis() + BURN_PROBE_EVERY_MS;
    }
    return s;
  }
  if (high && millis() >= burnChaosAtMs) {
    burnChaosEndMs = millis() + BURN_CHAOS_MS;
    burnChaosAtMs = millis() + BURN_CHAOS_EVERY_MS;
    return randomSegment();
  }
  return fullDutySegment(high);
}

// Segment i of block b, or false once the block is done.
bool blockSegmentAt(const BlockSpec &b, uint32_t i, Segment &s) {
  switch (b.block) {
    case B_CHAOS:
      if (millis() - blockStartMs >= b.durationMs) return false;
      s = randomSegment();
      return true;

    case B_LAPS:
    case B_HEAVY_LAPS:
    case B_CHECK_LAPS:
    case B_REGEN_BURSTS: {
      const Lap *laps = LAPS;
      size_t count = LAP_COUNT;
      if (b.block == B_HEAVY_LAPS) laps = HEAVY_LAPS, count = HEAVY_LAP_COUNT;
      if (b.block == B_CHECK_LAPS) laps = CHECK_LAPS, count = CHECK_LAP_COUNT;
      if (b.block == B_REGEN_BURSTS) laps = REGEN_BURSTS, count = REGEN_BURST_COUNT;
      if (i >= count) return false;
      s.mode = MOTOR;
      s.bounce = b.block == B_LAPS;  // Heavy tables ramp their stops.
      s.command = laps[i].duty;
      s.holdMs = laps[i].holdMs;
      return true;
    }

    case B_BANG:
      if (i == 0) {
        s = moveSegment(BANG_LO_DEG, 2000, 3000);
      } else if (i == 1) {
        s = moveSegment(BANG_HI_GOAL, 1, BANG_TIMEOUT_MS);
        s.streaming = true;
        bangUp = true;
        bangStrokes = 0;
      } else if (i == 2) {
        s = moveSegment(BANG_LO_GOAL, 1, 1500);
      } else if (i == 3) {
        s = moveSegment(0, 3000, 4000);
      } else {
        return false;
      }
      return true;

    case B_PROBE:
      if (i >= PROBE_SEGMENTS) return false;
      s = probeSegment(i);
      return true;

    case B_BURN:
    case B_BURN_HIGH:
      s = burnSegment(i, b.block == B_BURN_HIGH);
      return true;

    case B_HEAT:
      if (steadyTemp >= (i == 0 ? HEAT_FLOOR_C : HEAT_TARGET_C)) return false;
      if (i == 0) {
        heatSide = -heatSide;
        s = moveSegment(heatSide * HEAT_LO_DEG, 1500, 2000);
      } else {
        s = moveSegment(heatSide * HEAT_HI_GOAL, 1, HEAT_SEGMENT_MS);
        s.streaming = true;
        bangUp = true;
      }
      return true;

    case B_CLIMB:
      if (i >= CLIMB_COUNT) return false;
      s = moveSegment(CLIMB[i].deg, CLIMB[i].moveMs, CLIMB[i].holdMs);
      return true;

    case B_SWINGS:
      if (i >= SWING_COUNT) return false;
      s = moveSegment(SWING_DEG[i], SWING_MOVE_MS, SWING_MOVE_MS + SWING_HOLD_MS);
      return true;

    case B_HOLDS: {
      constexpr uint32_t sweep = 2 * HOLD_MAX_DEG / HOLD_STEP_DEG + 1;  // Holds per direction.
      if (i >= 2 * sweep) return false;
      const int deg = i < sweep
        ? -HOLD_MAX_DEG + static_cast<int>(i) * HOLD_STEP_DEG
        : HOLD_MAX_DEG - static_cast<int>(i - sweep) * HOLD_STEP_DEG;
      s = moveSegment(deg, HOLD_MOVE_MS, HOLD_MS);
      return true;
    }

    case B_BACKLASH: {
      if (i >= BACKLASH_COUNT) return false;
      const float centre = BACKLASH_CENTRES_DEG[i / BACKLASH_PER_CENTRE];
      const size_t j = i % BACKLASH_PER_CENTRE;
      if (j == 0) {
        s = moveSegment(centre, 1000, 2500);
      } else {
        const float step = BACKLASH_STEPS_DEG[(j - 1) / 4];
        const float offsets[] = {step, 0, -step, 0};
        s = moveSegment(centre + offsets[(j - 1) % 4], BACKLASH_MOVE_MS, BACKLASH_HOLD_MS);
      }
      return true;
    }

    case B_CHIRP:
    case B_CHIRP_SOFT:
    case B_DUAL_SINE:
    case B_DUAL_SINE_SOFT:
    case B_RAISE_LOWER:
      if (i == 0) {
        s = moveSegment(0, 1000, 2500);  // Start from hanging still.
        return true;
      }
      if (i == 1) {
        s = moveSegment(0, TRAJ_TIME_MS, b.durationMs);
        s.streaming = true;
        return true;
      }
      return false;

    case B_LIFT_DROP:
      if (i >= 2 * DROP_COUNT) return false;
      if (i % 2 == 0) {
        s = moveSegment(DROP_DEG[i / 2], 1000, 2000);
      } else {
        s.mode = POSITION;
        s.release = true;
        s.holdMs = DROP_RELEASE_MS;
      }
      return true;

    default:
      return false;
  }
}

Segment planSegment() {
  while (blockIndex < PLAN_COUNT) {
    const BlockSpec &b = PLAN[blockIndex];
    if (blockSegment == 0) {
      blockStartMs = millis();
      setKp(b.kp);
    }

    Segment s{};
    if (blockSegmentAt(b, blockSegment++, s)) return s;

    ++blockIndex;
    blockSegment = 0;
  }

  // Plan done: default gain, parked hanging down.
  setKp(DEFAULT_KP);
  return moveSegment(0, PARK_MOVE_MS, UINT32_MAX);
}

Segment nextSegment() {
  if (!havePending) {
    pending = planSegment();
    havePending = true;
    creepSinceMs = millis();
  }
  const bool outside = lastPosition < GOAL_MIN || lastPosition > GOAL_MAX;
  if (pending.mode == POSITION && !pending.release && outside &&
      millis() - creepSinceMs < CREEP_MAX_MS) {
    Segment s{};
    s.mode = MOTOR;
    s.command = lastPosition > GOAL_MAX ? CREEP_DUTY : -CREEP_DUTY;  // +PWM: counts down
    s.holdMs = 100;
    return s;
  }
  havePending = false;
  return pending;
}

void applySegment() {
  segmentModeEntry = false;
  stalled = false;
  braking = false;
  appliedDuty = segment.command;

  if (segment.release) {
    setTorque(false);
    released = true;
    return;
  }

  if (released) {
    // The shaft moved with torque off. Leave position mode so re-entering
    // it below resets the stale goal to the present position first.
    servoWrite(0x09, MOTOR_MODE, sizeof(MOTOR_MODE));
    servoMode = MOTOR;
    segmentModeEntry = true;
  }

  if (segment.mode == MOTOR) {
    if (segment.bounce && servoMode == MOTOR) {
      servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));
    }
    // Entering motor mode clears the PWM register, so the duty goes last.
    if (segment.bounce || servoMode != MOTOR) {
      servoWrite(0x09, MOTOR_MODE, sizeof(MOTOR_MODE));
      segmentModeEntry = true;
    }
    setDuty(segment.command);
  } else {
    if (servoMode != POSITION) {
      servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));
      segmentModeEntry = true;
    }
    // Entering position mode resets the goal to the present position,
    // so the goal is written after the switch.
    writeGoal(segment.command, segment.commandTime);
  }

  servoMode = segment.mode;
  if (released) {
    setTorque(true);
    released = false;
  }
}

bool inBlock(Block b);

// Flips the goal when the pendulum passes a flip point; a bang segment
// ends after the last stroke, a heat segment on temperature or time.
void bangStep() {
  const bool heat = inBlock(B_HEAT);
  const int side = heat ? heatSide : 1;
  const float deg = side * (static_cast<int>(lastPosition) - DOWN_COUNT) / COUNTS_PER_DEG;
  if (bangUp && deg >= (heat ? HEAT_HI_DEG : BANG_HI_DEG)) {
    bangUp = false;
    ++bangStrokes;
  } else if (!bangUp && deg <= (heat ? HEAT_LO_DEG : BANG_LO_DEG)) {
    bangUp = true;
    ++bangStrokes;
  }
  if (!heat && bangStrokes >= BANG_STROKES) {
    segment.holdMs = millis() - segmentStartMs;
    return;
  }
  const float goalDeg = heat ? (bangUp ? HEAT_HI_GOAL : HEAT_LO_GOAL)
                             : (bangUp ? BANG_HI_GOAL : BANG_LO_GOAL);
  const int16_t goal = goalCount(side * goalDeg);
  if (goal != segment.command) {
    segment.command = goal;
    writeGoal(goal, 1);
  }
}

void streamTrajectory() {
  if (PLAN[blockIndex].block == B_BANG || PLAN[blockIndex].block == B_HEAT) {
    bangStep();
    return;
  }
  const float t = (millis() - segmentStartMs) / 1000.0f;
  segment.command = goalCount(trajectoryDeg(PLAN[blockIndex].block, t));
  writeGoal(segment.command, TRAJ_TIME_MS);
}

// Flat out against the motion while falling through the lower half,
// otherwise the segment's duty. Positive PWM drives the count down.
void brakeStep() {
  const int offset = static_cast<int>(lastPosition) - DOWN_COUNT;
  const bool falling = abs(offset) <= BRAKE_COUNTS && abs(lastSpeed) >= BRAKE_MIN_SPEED &&
                       abs(positionStep) <= BRAKE_MAX_STEP && offset * lastSpeed < 0;
  if (falling == braking) return;
  braking = falling;

  const int16_t duty = braking ? (lastSpeed > 0 ? MAX_DUTY : -MAX_DUTY) : segment.command;
  if (duty == appliedDuty) return;
  servoWrite(0x09, POSITION_MODE, sizeof(POSITION_MODE));
  servoWrite(0x09, MOTOR_MODE, sizeof(MOTOR_MODE));
  setDuty(duty);
  appliedDuty = duty;
  rampReset = true;
}

bool inBlock(Block b) {
  return blockIndex < PLAN_COUNT && PLAN[blockIndex].block == b;
}

bool burning() {
  return inBlock(B_BURN) || inBlock(B_BURN_HIGH);
}

void updateCooling(uint8_t temperature) {
  if (burning()) return;  // The servo's own protection is the limit.
  static uint8_t streak = 0;
  const uint8_t hot = hotPlan ? HOT_RUN_HOT_C : HOT_C;
  const uint8_t cool = hotPlan ? HOT_RUN_COOL_C : COOL_C;
  const bool past = cooling ? temperature <= cool : temperature >= hot;
  streak = past ? streak + 1 : 0;
  if (streak < TEMP_READS) return;
  streak = 0;

  if (!cooling) {
    setTorque(false);
    cooling = true;
  } else {
    setTorque(true);
    cooling = false;
    servoMode = COOLDOWN;                         // Force a mode write...
    segmentStartMs = millis() - segment.holdMs;   // ...on a new segment now.
  }
}

uint8_t median3(const uint8_t *v) {
  return max(min(v[0], v[1]), min(max(v[0], v[1]), v[2]));
}

void updateTemperature(uint8_t temperature) {
  recentTemps[tempCount++ % 3] = temperature;
  steadyTemp = median3(recentTemps);
  if (inBlock(B_HEAT) && steadyTemp >= HEAT_TARGET_C) {
    segment.holdMs = millis() - segmentStartMs;  // Warm enough: end the block now.
  }
}

void updateBurn(uint8_t temperature, uint8_t error) {
  static uint8_t streak = 0;
  if (burnTripped || !burning()) return;
  const bool hot = (error & ERROR_OVERHEAT) || temperature >= BURN_MAX_C;
  streak = hot ? streak + 1 : 0;
  if (streak < TEMP_READS) return;
  burnTripped = true;
  segment.holdMs = millis() - segmentStartMs + BURN_WATCH_MS;  // The release follows.
}

// Returns true if the trip found a dead drive.
bool updateStallGuard(uint16_t position, int16_t load, uint8_t voltage) {
  static uint8_t idleVoltage = 0;
  if (abs(load) < 30) idleVoltage = voltage;
  if (STALL_GUARD_MS == 0 || released) return false;

  const int16_t minLoad = segment.mode == MOTOR ? STALL_MIN_LOAD : POSITION_STALL_MIN_LOAD;
  const bool driven = abs(load) >= minLoad;
  const bool sensed = position >= 8 && position <= 1014;  // Not pinned in the blind spot.
  if (!driven || !sensed || abs(static_cast<int>(position) - stallPosition) > 2) {
    stallPosition = position;
    stallSinceMs = millis();
    return false;
  }

  if (millis() - stallSinceMs < STALL_GUARD_MS) return false;
  setTorque(false);
  released = true;  // The next segment turns torque back on.
  stalled = true;
  segment.holdMs = millis() - segmentStartMs;  // End this segment now.
  const bool dead = abs(load) >= DEAD_DRIVE_MIN_LOAD && voltage + 1 >= idleVoltage;
  if (dead) cooling = true;  // Back on at the cooldown temperature.
  return dead;
}

void sendFrame(uint8_t flags) {
  Frame f{};
  f.sync[0] = 0xA5;
  f.sync[1] = 0x5A;
  f.timeUs = micros();
  f.segment = segmentIndex;
  f.command = segment.brake ? appliedDuty : segment.command;
  f.commandTime = segment.commandTime;
  f.block = blockIndex < PLAN_COUNT ? PLAN[blockIndex].block : B_DONE;
  f.kp = currentKp;

  // 0x38..0x3F: position, speed, load, voltage, temperature.
  uint8_t d[8];
  const int error = servoRead(SERVO_ID, 0x38, sizeof(d), d);

  if (error < 0) {
    flags |= FLAG_READ_FAILED;
  } else {
    f.position = d[0] << 8 | d[1];
    f.speed = signMagnitude(d[2] << 8 | d[3], 15);
    f.load = signMagnitude(d[4] << 8 | d[5], 10);
    f.voltage = d[6];
    f.temperature = d[7];
    positionStep = static_cast<int>(f.position) - lastPosition;
    lastPosition = f.position;
    lastSpeed = f.speed;
    f.error = static_cast<uint8_t>(error);
    updateTemperature(f.temperature);
    updateCooling(f.temperature);
    updateBurn(f.temperature, f.error);
    if (updateStallGuard(f.position, f.load, f.voltage)) flags |= FLAG_DEAD_DRIVE;
  }
  f.mode = cooling ? COOLDOWN : stalled ? FAULT : segment.release ? RELEASED : segment.mode;
  f.flags = flags;

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&f);
  uint8_t checksum = 0;
  for (size_t i = 2; i < sizeof(f) - 1; ++i) {
    checksum += bytes[i];
  }
  f.checksum = checksum;

  Serial.write(bytes, sizeof(f));
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);

  // Same RX and TX pin selects open-drain one-wire UART.
  // Keep the external pull-up to 3.3V.
  // Do not call pinMode() on DATA_PIN after this.
  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, DATA_PIN, DATA_PIN);

  delay(1000);  // Allow the servo to finish powering up.

  const uint32_t seed = SEED ? SEED : (esp_random() | 1);
  rngState = seed;

  Serial.printf("# chaos v2 seed=%u sample_us=%u frame_bytes=%u servo_id=%u down=%u counts_per_rev=%.0f\n",
                static_cast<unsigned>(seed),
                static_cast<unsigned>(SAMPLE_US),
                static_cast<unsigned>(sizeof(Frame)),
                static_cast<unsigned>(SERVO_ID),
                static_cast<unsigned>(DOWN_COUNT),
                COUNTS_PER_DEG * 360);
  Serial.print("# blocks:");
  for (size_t i = 0; i <= B_DONE; ++i) {
    Serial.printf(" %u=%s", static_cast<unsigned>(i), BLOCK_NAMES[i]);
  }
  Serial.printf("\n# trajectories: %s", TRAJECTORY_INFO);
  Serial.print("\n# plan:");
  for (const BlockSpec &b : PLAN) {
    Serial.printf(" %s/kp%u", BLOCK_NAMES[b.block], b.kp);
  }
  Serial.println();
  Serial.printf("# plan start: block %u\n", static_cast<unsigned>(START_BLOCK));
  for (const BlockSpec &b : PLAN) {
    hotPlan = hotPlan || b.block == B_HEAT;
  }
  Serial.printf("# thermal: torque off at %u C until %u C", hotPlan ? HOT_RUN_HOT_C : HOT_C,
                hotPlan ? HOT_RUN_COOL_C : COOL_C);
  if (hotPlan) Serial.printf("; heat blocks run below %u C, up to %u C", HEAT_FLOOR_C, HEAT_TARGET_C);
  Serial.println();
  dumpRegisters();
  uint8_t temperature;
  if (servoRead(SERVO_ID, 0x3F, 1, &temperature) >= 0) {
    for (int i = 0; i < 3; ++i) updateTemperature(temperature);
  }
  Serial.println("# BEGIN");
  Serial.flush();

  setTorque(true);
  nextSampleUs = micros();
}

void loop() {
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextSampleUs) < 0) return;

  nextSampleUs += SAMPLE_US;
  if (static_cast<int32_t>(nowUs - nextSampleUs) >= 0) {
    nextSampleUs = nowUs + SAMPLE_US;  // Fell a whole period behind.
  }

  uint8_t flags = 0;
  if (!cooling && millis() - segmentStartMs >= segment.holdMs) {
    segment = nextSegment();
    ++segmentIndex;
    applySegment();
    segmentStartMs = millis();

    flags |= FLAG_SEGMENT_START;
    if (segmentModeEntry) flags |= FLAG_MODE_ENTRY;
  }
  if (rampReset) {
    flags |= FLAG_MODE_ENTRY;
    rampReset = false;
  }

  sendFrame(flags);

  if (segment.brake && !cooling && !released) {
    brakeStep();
  }

  // A goal written every sample just before the read made the servo miss
  // most reads, so the next goal goes out right after this sample's read.
  if (segment.streaming && !cooling && !stalled) {
    streamTrajectory();
  }
}
