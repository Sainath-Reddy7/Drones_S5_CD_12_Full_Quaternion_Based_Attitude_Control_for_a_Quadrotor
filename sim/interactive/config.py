"""Configuration for the interactive simulator.

The DRONE + CONTROLLER sections are FROZEN project values (the paper's) --
they are exposed here for visibility, not tuning. Everything under GUIDANCE /
SAFETY / ENVIRONMENT is interactive-app scaffolding around the frozen law.
"""

# ---- FROZEN: the paper's vehicle + control law (do not tune) -----------
DRONE = {
    "mass_kg": 0.2,
    "inertia": (6.5e-4, 6.5e-4, 1.2e-3),  # Ixx, Iyy, Izz
    "arm_m": 0.15,
    "thrust_max_total_n": 4 * 1.95,
}
CONTROLLER = {
    "Pq": 20.0,          # paper Section V
    "Pw": 4.0,
    "torque_limit_nm": 4.0,
    "control_hz": 1000,  # controller tick (ZOH between ticks)
    "physics_hz": 10_000,
}

# ---- guidance scaffolding (NOT the paper law) ---------------------------
GUIDANCE = {
    "pos_k": 1.1,        # position error -> desired velocity
    "vel_k": 1.6,        # velocity error -> acceleration
    "acc_max": 2.8,      # m/s^2 horizontal acceleration clamp
    "v_max": 3.0,        # m/s horizontal cruise clamp (must exceed path speeds)
    "vz_clamp": 1.2,     # m/s vertical
    "max_tilt_rad": 0.45,
    "yaw_rate": 1.3,
    "climb": 1.0,        # m/s pilot climb
    "wp_capture_r": 0.8, # waypoint capture radius
}

# ---- paper-test protocol (matches the benchmark adapters) ---------------
PAPER_TESTS = {
    "step": {"duration": 15.0},
    "sine": {"duration": 15.0},
    "flip": {"duration": 6.0, "start_alt": 10.0, "idle_fraction": 0.1},
}

# ---- path tracking ------------------------------------------------------
PATHS = {
    "mission": [  # city tour over the road corridor
        (0.0, 6.0, 3.0), (0.0, 26.0, 4.5), (7.0, 30.0, 5.5),
        (0.0, -2.0, 3.5), (0.0, -6.0, 3.0),
    ],
    "circle": {"center": (0.0, 12.0, 4.0), "radius": 8.0, "omega": 0.3},  # 2.4 m/s tangential
    "figure8": {"center": (0.0, 14.0, 5.0), "a": 9.0, "b": 6.0, "omega": 0.3},  # ~2.7 m/s peak
    "straight": {"start": (0.0, -6.0, 3.0), "end": (0.0, 30.0, 4.5), "speed": 1.5},
}

# ---- safety limits ------------------------------------------------------
SAFETY = {
    "max_alt": 35.0,
    "min_alt": 0.25,
    "max_speed": 8.0,
    "max_tilt_deg": 50.0,
}

# ---- environment / sensors ----------------------------------------------
WIND_LEVELS = {  # base horizontal acceleration (m/s^2) + gust amplitude
    "OFF": (0.0, 0.0),
    "LOW": (0.15, 0.08),
    "MEDIUM": (0.4, 0.2),
    "HIGH": (0.8, 0.4),
}
NOISE_LEVELS = {  # uniform amplitude on quaternion components + body rates
    "PERFECT": 0.0,
    "LOW": 0.02,
    "PAPER": 0.1,   # the paper's own sensor model
    "HIGH": 0.25,
}
TRAFFIC = {  # kinematic cars (controlled kinematics, real contacts)
    "speed_m_s": 2.0,
    "lane_len": 70.0,
}
