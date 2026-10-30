"""SC15 actuator model: firmware controller, motor, friction, pendulum.

One tick is the servo's 4 ms update. Per tick the model reads its sensor
(whole counts, pinned in the blind spot; speed = the count change from
7 to 2 ticks back over 20 ms, quantized to 50 counts/s, as the register
matches),
runs the measured firmware law, slew-limits the output, and integrates
the pendulum in SUBSTEPS steps:

  goal            ramps linearly from the last goal to a new one over the goal
                  time (whole ticks; streamed 4 ms goals apply at once); entering
                  position mode resets it to the present position
  position mode   u* = -sign(e) (PUNCH + kp (|e| - 1)) for |e| > dead band,
                  plus kd * speed / 250; output builds <= 20/tick, falls <= 400/tick
  motor mode      u* = commanded duty; output slews 5/tick either way
  mode entry      output resets to 0 (flag 0x02)
  torque off      motor disconnected (no back-EMF braking)

  motor torque    f(T) (k (1 +- k_dir) u V - b w), with u in -1..1 (+u drives the angle up),
                  V the supply, f(T) = 1 / (1 + aT (T - 25))
  friction        BAM M1..M6 (Duclusaud et al., ICRA 2025), clipped to what
                  stops the joint within the substep
  pendulum        J = J_load + spool J_spool + J_m, gravity -mgl sin(q + offset)

Angles in rad from hanging down, positive where the count rises. Positive
PWM in the logs drives the count down, so u = -load / 1000.
"""

import math

import numpy as np
from numba import njit, prange

COUNTS_PER_RAD = 1610 / (2 * math.pi)
DOWN_COUNT = 513
TICK = 0.004
SUBSTEPS = 4
PUNCH = 30.0
SPEED_LAG = 2   # ticks
SPEED_SPAN = 5  # ticks

# name, low, high, log scale. Order is the parameter vector.
PARAMS = [
    ('k', 0.05, 1.0, True),          # N·m per volt of duty x supply
    ('b', 0.01, 2.0, True),          # back-EMF damping, N·m·s/rad
    ('J_m', 1e-4, 0.05, True),       # reflected rotor inertia, kg·m²
    ('J_spool', 1e-5, 0.01, True),   # 1.52 kg spool about its own centre
    ('offset', -0.05, 0.05, False),  # rad
    ('aT', 0.0, 0.03, False),        # per °C reported
    ('kd', 20.0, 150.0, False),      # firmware D, PWM per count/tick
    ('Kc', 1e-4, 0.5, True),
    ('Kv', 1e-4, 1.0, True),
    ('Kcs', 1e-4, 0.5, True),
    ('vs', 0.01, 3.0, True),
    ('alpha', 0.5, 10.0, False),
    ('Km', 1e-4, 1.0, True),
    ('Ke', 1e-4, 1.0, True),
    ('Kms', 1e-4, 1.0, True),
    ('Kes', 1e-4, 1.0, True),
    ('Kmq', 1e-4, 1.0, True),
    ('Keq', 1e-4, 1.0, True),
    ('k_dir', -0.3, 0.3, False),    # k (1 + k_dir) driving the angle up, k (1 - k_dir) down
]
NAMES = [p[0] for p in PARAMS]
IDX = {n: i for i, n in enumerate(NAMES)}

# Which parameters each model frees; the rest are pinned to "off".
COMMON = ['k', 'b', 'J_m', 'J_spool', 'offset', 'kd']
MODELS = {
    'm1': COMMON + ['Kc', 'Kv'],
    'm2': COMMON + ['Kc', 'Kv', 'Kcs', 'vs', 'alpha'],
    'm3': COMMON + ['Kc', 'Kv', 'Km'],                                   # Ke tied to Km
    'm4': COMMON + ['Kc', 'Kv', 'Kcs', 'vs', 'alpha', 'Km', 'Kms'],      # Ke, Kes tied
    'm5': COMMON + ['Kc', 'Kv', 'Kcs', 'vs', 'alpha', 'Km', 'Ke', 'Kms', 'Kes'],
    'm6': COMMON + ['Kc', 'Kv', 'Kcs', 'vs', 'alpha', 'Km', 'Ke', 'Kms', 'Kes', 'Kmq', 'Keq'],
}
TIED = {'m3': [('Ke', 'Km')], 'm4': [('Ke', 'Km'), ('Kes', 'Kms')]}
OFF = {'k_dir': 0.0, 'aT': 0.0, 'Kcs': 0.0, 'vs': 1.0, 'alpha': 2.0, 'Km': 0.0, 'Ke': 0.0,
       'Kms': 0.0, 'Kes': 0.0, 'Kmq': 0.0, 'Keq': 0.0}


def full_vector(model, free, thermal=False, directional=False):
    """Parameter vector from a dict of free values, with ties and offs applied."""
    p = np.array([OFF.get(n, np.nan) for n in NAMES])
    for n, v in free.items():
        p[IDX[n]] = v
    for dst, src in TIED.get(model, []):
        p[IDX[dst]] = p[IDX[src]]
    if not thermal:
        p[IDX['aT']] = 0.0
    if not directional:
        p[IDX['k_dir']] = 0.0
    return p


@njit(cache=True)
def sense(q):
    """Count the potentiometer reports: whole counts, pinned in the blind spot."""
    c = DOWN_COUNT + q * COUNTS_PER_RAD
    c = c - 1610.0 * math.floor(c / 1610.0)
    c = math.floor(c + 0.5)
    if c > 1023.0:
        return 1022.0 if c < 1316.0 else 0.0
    return c


@njit(cache=True)
def friction(p, tm, te, w):
    """BAM friction magnitude for motor torque tm, external torque te, speed w."""
    f = p[8] * abs(w) + p[7] + abs(p[12] * tm - p[13] * te)
    s = math.exp(-abs(w / p[10]) ** p[11])
    q = p[17] * te * te if abs(tm) > abs(te) else p[16] * tm * tm
    return f + s * (p[9] + abs(p[14] * tm - p[15] * te) + q)


@njit(cache=True)
def step_output(u, target, mode):
    """Firmware slew of the applied PWM towards target (PWM units, per tick)."""
    if mode == 0:
        d = target - u
        return u + max(-5.0, min(5.0, d))
    if u == 0.0 or (target > 0) == (u > 0):
        if abs(target) > abs(u):
            return math.copysign(min(abs(target), abs(u) + 20.0), target)
        return math.copysign(max(abs(target), abs(u) - 400.0), target) if target != 0 else \
            math.copysign(max(0.0, abs(u) - 400.0), u)
    if abs(u) > 400.0:
        return u - math.copysign(400.0, u)
    return math.copysign(min(abs(target), 20.0), target)


@njit(cache=True)
def rollout(p, mode, command, command_time, flags, load, volt, tempc, kp, mgl, jload, spool,
            q0, dq0, u0, closed_loop, out_q, meas, meas_vel, reset):
    """Simulates one window; writes the reported angle (rad) per tick to out_q.

    Every reset ticks (0: never) the angle and speed are re-synced to the
    measurement, as BAM's reset_period, for open-loop motor-mode windows.
    """
    J = jload + spool * p[3] + p[2]
    off = p[4]
    q, w, u = q0, dq0, u0
    n = mode.shape[0]
    dt = TICK / SUBSTEPS
    hist = np.empty(SPEED_LAG + SPEED_SPAN + 1)  # sensed counts, newest last
    for j in range(hist.shape[0]):
        hist[j] = sense(q0 - dq0 * TICK * (hist.shape[0] - 1 - j))
    goal = goal_from = goal_to = sense(q0)
    ramp_k = ramp_n = 1
    prev_mode = mode[0]
    for k in range(n):
        if reset > 0 and k > 0 and k % reset == 0 and not math.isnan(meas[k]) and not math.isnan(meas_vel[k]):
            q, w = meas[k], meas_vel[k]
        c = sense(q)
        hist[:-1] = hist[1:]
        hist[-1] = c
        out_q[k] = (c - DOWN_COUNT) / COUNTS_PER_RAD
        m = mode[k]
        on = m == 0 or m == 1
        if k > 0:
            if not on:
                u = 0.0
            elif closed_loop:
                if flags[k] & 2:
                    u = 0.0
                if m == 1:
                    if (flags[k] & 2) or prev_mode != 1:
                        goal = goal_to = c
                    if command[k] != goal_to:
                        goal_from, goal_to = goal, command[k]
                        ramp_n = max(1, int(command_time[k] / (TICK * 1000.0) + 0.5))
                        ramp_k = 0
                    if ramp_k < ramp_n:
                        ramp_k += 1
                        goal = goal_from + (goal_to - goal_from) * ramp_k / ramp_n
                    e = goal - c
                    db = 1.0 if kp >= 30 else 2.0
                    t = 0.0
                    if abs(e) > db:
                        t = -math.copysign(min(PUNCH + kp * (abs(e) - 1.0), 1000.0), e)
                    newer = hist[hist.shape[0] - 1 - SPEED_LAG]
                    older = hist[0]
                    spd = 50.0 * math.floor((newer - older) / (SPEED_SPAN * TICK) / 50.0 + 0.5)
                    t += p[6] * spd / 250.0
                    t = max(-1000.0, min(1000.0, t))
                else:
                    t = command[k]
                u = step_output(u, t, m)
            else:
                u = load[k]
        prev_mode = m
        f_t = 1.0 / (1.0 + p[5] * (tempc[k] - 25.0))
        drive = -u / 1000.0 * volt[k] * p[0]
        drive *= 1.0 + math.copysign(p[18], drive)
        for _ in range(SUBSTEPS):
            tm = f_t * (drive - p[1] * w) if on else 0.0
            te = -mgl * math.sin(q + off)
            fmax = friction(p, tm, te, w)
            stop = -(J / dt * w + tm + te)
            tf = max(-fmax, min(fmax, stop))
            w += (tm + te + tf) / J * dt
            q += w * dt


@njit(parallel=True, cache=True)
def score(p, starts, lengths, mode, command, command_time, flags, load, volt, tempc, meas, valid, meas_vel,
          kp, mgl, jload, spool, q0, dq0, u0, reset, closed_loop, err_out):
    """Mean absolute angle error per window (rad), into err_out."""
    for i in prange(starts.shape[0]):
        a, n = starts[i], lengths[i]
        sim = np.empty(n)
        rollout(p, mode[a:a + n], command[a:a + n], command_time[a:a + n], flags[a:a + n], load[a:a + n],
                volt[a:a + n], tempc[a:a + n], kp[i], mgl[i], jload[i], spool[i],
                q0[i], dq0[i], u0[i], closed_loop, sim, meas[a:a + n], meas_vel[a:a + n], reset[i])
        tot, cnt = 0.0, 0
        for k in range(1, n):
            if valid[a + k]:
                d = sim[k] - meas[a + k]
                d = d - 2 * math.pi * math.floor(d / (2 * math.pi) + 0.5)
                tot += abs(d)
                cnt += 1
        err_out[i] = tot / cnt if cnt else np.nan
