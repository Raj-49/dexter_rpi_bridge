"""
servo_config.py
===============
Direct translation of the ESP32 firmware SERVO_CONFIG[] table.
All PWM values are in microseconds. All angles are in radians.

Joint index mapping (matches dexter_hardware_interface.cpp):
  [0]  jl1  Left  Base rotation        (PCA ch0,  RDS3235 heavy)
  [1]  jl2  Left  Shoulder lift        (PCA ch1,  RDS3235 heavy)
  [2]  jl3  Left  Elbow bend     [INV] (PCA ch2,  RDS3235 heavy)
  [3]  jl4  Left  Wrist pitch          (PCA ch6,  RKI-1206 light)
  [4]  jl5  Left  Wrist roll           (PCA ch7,  RKI-1206 light)
  [5]  jl6  Left  End rotation   [INV] (PCA ch8,  RKI-1206 light)
  [6]  jl7  Left  Gripper              (PCA ch9,  RKI-1206 light)
  [7]  jr1  Right Base rotation        (PCA ch3,  RDS3235 heavy)
  [8]  jr2  Right Shoulder lift  [INV] (PCA ch4,  RDS3235 heavy)
  [9]  jr3  Right Elbow bend     [INV] (PCA ch5,  RDS3235 heavy)
  [10] jr4  Right Wrist pitch    [INV] (PCA ch10, RKI-1206 light)
  [11] jr5  Right Wrist roll     [INV] (PCA ch11, RKI-1206 light)
  [12] jr6  Right End rotation         (PCA ch12, RKI-1206 light)
  [13] jr7  Right Gripper              (PCA ch13, RKI-1206 light)
"""

import math

# ── Servo model range constants ───────────────────────────────────────────────
# RDS3235 (35kg heavy, joints 0-2 and 7-9):
#   180° model: 500-2500µs → ±1000µs for ±90°  → RANGE_90_HEAVY = 1000
#   270° model: 500-2500µs → ±667µs  for ±90°  → RANGE_90_HEAVY = 667
RANGE_90_HEAVY: int = 1000

# RKI-1206 (15kg light, joints 3-6 and 10-13):
#   180° model: 550-2400µs → ±925µs  for ±90°  → RANGE_90_LIGHT = 925
RANGE_90_LIGHT: int = 925

NUM_JOINTS: int = 14

# ── Per-joint configuration ────────────────────────────────────────────────────
# Each entry: (min_pwm, max_pwm, min_rad, max_rad, init_pwm, pca_channel, name)
SERVO_CONFIG: list[tuple] = [
    # ── LEFT ARM: PCA ch 0-2, RDS3235 35kg heavy servos ─────────────────────
    (1550 - RANGE_90_HEAVY, 1550 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 1550,  0, 'jl1'),
    (1750 - RANGE_90_HEAVY, 1750 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 1750,  1, 'jl2'),
    (1575 - RANGE_90_HEAVY, 1575 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 1575,  2, 'jl3'),

    # ── LEFT WRIST + GRIPPER: PCA ch 6-9, RKI-1206 15kg light servos ────────
    (1725 - RANGE_90_LIGHT, 1725 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 1725,  6, 'jl4'),
    (1452 - RANGE_90_LIGHT, 1452 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 1452,  7, 'jl5'),
    (1848 - RANGE_90_LIGHT, 1848 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 1848,  8, 'jl6'),
    (500,                    2400,                   0.0,        math.pi,    500,   9, 'jl7'),  # Gripper

    # ── RIGHT ARM: PCA ch 3-5, RDS3235 35kg heavy servos ────────────────────
    (2076 - RANGE_90_HEAVY, 2076 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 2076,  3, 'jr1'),
    (2200 - RANGE_90_HEAVY, 2200 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 2200,  4, 'jr2'),
    (1200 - RANGE_90_HEAVY, 1200 + RANGE_90_HEAVY, -math.pi/2, math.pi/2, 1200,  5, 'jr3'),

    # ── RIGHT WRIST + GRIPPER: PCA ch 10-13, RKI-1206 15kg light servos ─────
    (1952 - RANGE_90_LIGHT, 1952 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 1952, 10, 'jr4'),
    (2300 - RANGE_90_LIGHT, 2300 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 2300, 11, 'jr5'),
    (1848 - RANGE_90_LIGHT, 1848 + RANGE_90_LIGHT, -math.pi/2, math.pi/2, 1848, 12, 'jr6'),
    (500,                    2400,                   0.0,        math.pi,    500,  13, 'jr7'),  # Gripper
]

# ── Inverted joints ───────────────────────────────────────────────────────────
# These joints have mechanically reversed servo mounting (Z-axis flip in URDF).
# Commands are negated before PWM conversion; feedback is negated after.
# Directly from firmware.ino is_inverted_joint():
INVERTED_JOINTS: frozenset[int] = frozenset({2, 5, 8, 9, 10, 11})

# ── Safety limits ─────────────────────────────────────────────────────────────
SOFT_LIMIT_RAD: float  = 1.500   # 86° — firmware will clamp beyond this
SOFT_BRAKE_RAD: float  = 1.449   # 83° — braking zone begins here
SERVO_DEADBAND_US: int = 4       # Min PWM change before writing to hardware

# ── Motion profile parameters ────────────────────────────────────────────────
# Tuned to match the ESP32 firmware's interpolation feel.
# Commands arrive at 25 Hz (40ms apart). The profile must reach each waypoint
# before the next one arrives, while still feeling smooth.
MAX_VELOCITY_US_PER_S: float     = 6000.0   # µs/s  (full range ~0.3s)
MAX_ACCEL_US_PER_S2: float       = 40000.0  # µs/s² — aggressive tracking for 50Hz commands
COMMAND_TIMEOUT_S: float         = 0.5      # Freeze servos after 500ms silence
CONTROL_HZ: int                  = 50       # Control loop rate (matches laptop 50Hz)

# ── Accessor helpers ─────────────────────────────────────────────────────────
def get_min_pwm(i: int) -> int:    return SERVO_CONFIG[i][0]
def get_max_pwm(i: int) -> int:    return SERVO_CONFIG[i][1]
def get_min_rad(i: int) -> float:  return SERVO_CONFIG[i][2]
def get_max_rad(i: int) -> float:  return SERVO_CONFIG[i][3]
def get_init_pwm(i: int) -> int:   return SERVO_CONFIG[i][4]
def get_channel(i: int) -> int:    return SERVO_CONFIG[i][5]
def get_name(i: int) -> str:       return SERVO_CONFIG[i][6]

def rad_to_pwm(rad: float, joint_idx: int) -> int:
    """Convert radians to PWM microseconds for a given joint."""
    cfg = SERVO_CONFIG[joint_idx]
    min_pwm, max_pwm, min_rad, max_rad = cfg[0], cfg[1], cfg[2], cfg[3]
    if max_rad == min_rad:
        return cfg[4]  # init_pwm fallback
    frac = (rad - min_rad) / (max_rad - min_rad)
    pwm = min_pwm + frac * (max_pwm - min_pwm)
    return int(max(float(min_pwm), min(float(max_pwm), pwm)))

def pwm_to_rad(pwm: float, joint_idx: int) -> float:
    """Convert PWM microseconds to radians for a given joint."""
    cfg = SERVO_CONFIG[joint_idx]
    min_pwm, max_pwm, min_rad, max_rad = cfg[0], cfg[1], cfg[2], cfg[3]
    if max_pwm == min_pwm:
        return 0.0
    frac = (pwm - min_pwm) / (max_pwm - min_pwm)
    return min_rad + frac * (max_rad - min_rad)
