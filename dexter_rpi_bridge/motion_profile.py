"""
motion_profile.py
=================
Per-joint trapezoidal motion profile.

Mirrors the ESP32 firmware's interpolation behaviour but with explicit
acceleration, cruise, and deceleration phases. This eliminates the
"launch and pause" jitter from burst UDP packets.

Phase state machine:
  IDLE  → no movement needed (at target within deadband)
  ACCEL → ramping velocity up toward MAX_VELOCITY
  CRUISE→ moving at MAX_VELOCITY
  DECEL → braking to arrive at target precisely

Parameters (tuned to match firmware feel):
  max_velocity_us_s   = 6000 µs/s  (full heavy-servo range in ~0.33s)
  max_accel_us_s2     = 15000 µs/s² (0→max_vel in ~0.4s)
"""

from enum import IntEnum


class Phase(IntEnum):
    IDLE   = 0
    ACCEL  = 1
    CRUISE = 2
    DECEL  = 3


class MotionProfile:
    """
    Trapezoidal motion profile for a single servo joint.
    All internal values are in PWM microseconds.
    """

    def __init__(
        self,
        init_pwm: int,
        max_velocity_us_s: float = 6000.0,
        max_accel_us_s2: float   = 15000.0,
        deadband_us: int          = 4,
    ) -> None:
        self.current_pwm: float  = float(init_pwm)
        self.target_pwm:  float  = float(init_pwm)
        self.velocity:    float  = 0.0           # µs/s, signed
        self.phase:       Phase  = Phase.IDLE

        self._max_vel    = max_velocity_us_s
        self._max_accel  = max_accel_us_s2
        self._deadband   = deadband_us

    # ── Public API ────────────────────────────────────────────────────────────

    def set_target(self, target_pwm: int) -> None:
        """Update the target. The profile will smoothly drive toward it."""
        self.target_pwm = float(target_pwm)
        # Re-evaluate phase on next step()
        if abs(self.target_pwm - self.current_pwm) > self._deadband:
            if self.phase == Phase.IDLE:
                self.phase = Phase.ACCEL

    def snap_to(self, target_pwm: int) -> None:
        """
        Immediately jump to target without interpolation.
        Used for the very first command on each joint to prevent
        a violent sweep from the home position.
        """
        self.current_pwm = float(target_pwm)
        self.target_pwm  = float(target_pwm)
        self.velocity    = 0.0
        self.phase       = Phase.IDLE

    def step(self, dt_s: float) -> int:
        """
        Advance the profile by dt_s seconds.
        Returns the new current PWM value as an integer.
        """
        error = self.target_pwm - self.current_pwm

        if abs(error) <= self._deadband:
            # Arrived — snap exactly and idle
            self.current_pwm = self.target_pwm
            self.velocity    = 0.0
            self.phase       = Phase.IDLE
            return int(round(self.current_pwm))

        direction = 1.0 if error > 0 else -1.0

        # Braking distance at current velocity: v² / (2 * a)
        stopping_dist = (self.velocity ** 2) / (2.0 * self._max_accel + 1e-12)

        if abs(error) <= stopping_dist + self._deadband:
            # ── DECEL phase ──────────────────────────────────────────────────
            self.phase = Phase.DECEL
            self.velocity -= direction * self._max_accel * dt_s
            # Prevent velocity reversal past zero
            if direction > 0 and self.velocity < 0.0:
                self.velocity = 0.0
            elif direction < 0 and self.velocity > 0.0:
                self.velocity = 0.0

        elif abs(self.velocity) < self._max_vel:
            # ── ACCEL phase ──────────────────────────────────────────────────
            self.phase = Phase.ACCEL
            self.velocity += direction * self._max_accel * dt_s
            # Clamp to max velocity
            if self.velocity > self._max_vel:
                self.velocity = self._max_vel
            elif self.velocity < -self._max_vel:
                self.velocity = -self._max_vel
        else:
            # ── CRUISE phase ─────────────────────────────────────────────────
            self.phase = Phase.CRUISE

        # Integrate position
        self.current_pwm += self.velocity * dt_s

        # Clamp to [current, target] to prevent overshoot
        if direction > 0:
            self.current_pwm = min(self.current_pwm, self.target_pwm)
        else:
            self.current_pwm = max(self.current_pwm, self.target_pwm)

        return int(round(self.current_pwm))

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (
            f'MotionProfile(cur={self.current_pwm:.0f}µs, '
            f'tgt={self.target_pwm:.0f}µs, '
            f'vel={self.velocity:.0f}µs/s, '
            f'phase={self.phase.name})'
        )
