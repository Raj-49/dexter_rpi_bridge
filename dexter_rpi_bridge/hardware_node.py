"""
hardware_node.py
================
Main ROS 2 node for the Dexter RPi hardware bridge.

Replaces the ESP32 firmware entirely. Runs on Raspberry Pi 3, connected
to PCA9685 servo driver boards via I2C.

Architecture (mirrors ESP32 dual-task design):
  ROS Thread  — rclpy.spin(), handles /rpi/joint_commands callback,
                runs state + health publishers via timers.
  Control Thread @ 100Hz — reads latest command from shared slot,
                steps motion profiles, writes PWM to PCA9685.

Topics:
  Subscribes:  /rpi/joint_commands  (Float64MultiArray, 15 elements)
  Publishes:   /rpi/joint_states    (Float64MultiArray, 14 elements) @ 100Hz
               /rpi/link_health     (Float64MultiArray, 25 elements) @   4Hz

Message format (identical to ESP32 firmware):
  /rpi/joint_commands: [j0..j13, seq_id]  — 14 joint angles + monotonic counter
  /rpi/joint_states:   [j0..j13]          — 14 joint angles (radians)
  /rpi/link_health:    [uptime_s, cmd_age_ms, rx_total, rx_accepted,
                        drop_bad_size, drop_invalid, drop_stale_seq,
                        timeout_events, last_rx_seq, cpu_temp_c, network_ok,
                        pwm0..pwm13]       — 25 fields total
"""

import math
import threading
import time
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Float64MultiArray

from .servo_config import (
    SERVO_CONFIG,
    INVERTED_JOINTS,
    SOFT_LIMIT_RAD,
    SERVO_DEADBAND_US,
    COMMAND_TIMEOUT_S,
    CONTROL_HZ,
    MAX_VELOCITY_US_PER_S,
    MAX_ACCEL_US_PER_S2,
    NUM_JOINTS,
    rad_to_pwm,
    pwm_to_rad,
    get_init_pwm,
    get_channel,
    get_name,
)
from .pca9685_driver import PCA9685Driver
from .motion_profile import MotionProfile

logger = logging.getLogger(__name__)


class DexterRpiBridgeNode(Node):
    """
    Hardware bridge node: /rpi/joint_commands → PCA9685 → /rpi/joint_states
    """

    _STATE_PUB_HZ:  int = 100
    _HEALTH_PUB_HZ: int = 4

    def __init__(self) -> None:
        super().__init__('dexter_rpi_bridge')

        # ── QoS: best_effort, depth 1 — matches DexterHardwareInterface ──────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(
            Float64MultiArray, '/rpi/joint_states', qos)
        self._health_pub = self.create_publisher(
            Float64MultiArray, '/rpi/link_health', qos)

        # ── Subscriber ────────────────────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/rpi/joint_commands',
            self._command_callback,
            qos,
        )

        # ── PCA9685 ───────────────────────────────────────────────────────────
        self._pca = PCA9685Driver(address=0x40, freq_hz=50)
        if not self._pca.is_ready:
            self.get_logger().error(
                '[PCA9685] Hardware not ready — running in DRY-RUN mode. '
                'No servo output will be produced.'
            )

        # ── Motion profiles (one per joint) ──────────────────────────────────
        self._profiles: list[MotionProfile] = [
            MotionProfile(
                init_pwm=get_init_pwm(i),
                max_velocity_us_s=MAX_VELOCITY_US_PER_S,
                max_accel_us_s2=MAX_ACCEL_US_PER_S2,
                deadband_us=SERVO_DEADBAND_US,
            )
            for i in range(NUM_JOINTS)
        ]

        # ── Shared state between control thread and ROS thread ────────────────
        self._state_lock = threading.Lock()
        self._current_pwm: list[float] = [float(get_init_pwm(i)) for i in range(NUM_JOINTS)]
        self._engaged: list[bool] = [False] * NUM_JOINTS

        # ── Latest command slot (written by ROS callback, read by control thread)
        self._cmd_lock = threading.Lock()
        self._latest_cmd: list[float] | None = None

        # ── Telemetry counters ────────────────────────────────────────────────
        self._rx_total:        int   = 0
        self._rx_accepted:     int   = 0
        self._drop_bad_size:   int   = 0
        self._drop_invalid:    int   = 0
        self._drop_stale_seq:  int   = 0
        self._timeout_events:  int   = 0
        self._last_rx_seq:     int   = -1
        self._seq_initialized: bool  = False
        self._last_cmd_time:   float | None = None
        self._cmd_stale:       bool  = False
        self._start_time:      float = time.monotonic()

        # ── Start control thread ──────────────────────────────────────────────
        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='dexter_control')
        self._control_thread.start()

        # ── ROS timers ────────────────────────────────────────────────────────
        self.create_timer(1.0 / self._STATE_PUB_HZ,  self._publish_state)
        self.create_timer(1.0 / self._HEALTH_PUB_HZ, self._publish_health)

        self.get_logger().info(
            '✓ dexter_rpi_bridge started\n'
            f'  PCA9685:  0x40 ({"READY" if self._pca.is_ready else "DRY-RUN"})\n'
            f'  Control:  {CONTROL_HZ}Hz\n'
            f'  Joints:   {NUM_JOINTS}\n'
            '  Waiting for /rpi/joint_commands...'
        )

    # ── ROS Callback ─────────────────────────────────────────────────────────

    def _command_callback(self, msg: Float64MultiArray) -> None:
        """Validate and store the latest joint command."""
        self._rx_total += 1

        # Size validation: need at least 14 joint values
        if len(msg.data) < NUM_JOINTS:
            self._drop_bad_size += 1
            return

        # Sequence ID validation (slot 14) — reject stale/out-of-order packets
        if len(msg.data) >= NUM_JOINTS + 1:
            seq_d = msg.data[NUM_JOINTS]
            if not math.isfinite(seq_d) or seq_d < 0.0:
                self._drop_invalid += 1
                return
            seq = int(seq_d)
            if self._seq_initialized and seq <= self._last_rx_seq:
                self._drop_stale_seq += 1
                return
            self._seq_initialized = True
            self._last_rx_seq = seq

        # Finite check on all joint values
        for i in range(NUM_JOINTS):
            if not math.isfinite(msg.data[i]):
                self._drop_invalid += 1
                return

        # ── Accept ────────────────────────────────────────────────────────────
        self._rx_accepted += 1
        self._last_cmd_time = time.monotonic()
        self._cmd_stale = False

        with self._cmd_lock:
            self._latest_cmd = list(msg.data[:NUM_JOINTS])

    # ── Control Thread (100Hz) ────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """
        High-frequency servo control loop. Runs on a dedicated thread.
        Pulls the latest command, steps motion profiles, writes PWM to PCA9685.
        """
        dt_s = 1.0 / CONTROL_HZ
        next_tick = time.monotonic()

        while self._running:
            next_tick += dt_s
            now = time.monotonic()

            # ── Command timeout detection ─────────────────────────────────────
            if self._last_cmd_time is not None:
                cmd_age = now - self._last_cmd_time
                if cmd_age > COMMAND_TIMEOUT_S and not self._cmd_stale:
                    self._timeout_events += 1
                    self._cmd_stale = True
                    # Freeze: snap all motion targets to current position
                    for i in range(NUM_JOINTS):
                        self._profiles[i].set_target(int(self._profiles[i].current_pwm))

            # ── Read latest command ───────────────────────────────────────────
            with self._cmd_lock:
                cmd = self._latest_cmd

            # ── Apply command to motion profiles ──────────────────────────────
            if cmd is not None and not self._cmd_stale:
                for i in range(NUM_JOINTS):
                    rad = float(cmd[i])

                    # Soft-limit clamp
                    rad = max(-SOFT_LIMIT_RAD, min(SOFT_LIMIT_RAD, rad))

                    # Inversion: joints with reversed physical mounting
                    if i in INVERTED_JOINTS:
                        rad = -rad

                    target_pwm = rad_to_pwm(rad, i)

                    if not self._engaged[i]:
                        # First command: snap to target to avoid violent home-to-target sweep
                        self._profiles[i].snap_to(target_pwm)
                        self._engaged[i] = True
                        continue

                    self._profiles[i].set_target(target_pwm)

            # ── Step all profiles and write PWM ───────────────────────────────
            new_pwm: list[float] = []
            for i in range(NUM_JOINTS):
                pwm = float(self._profiles[i].step(dt_s))
                new_pwm.append(pwm)

                if self._engaged[i]:
                    prev = self._current_pwm[i]
                    if abs(pwm - prev) > SERVO_DEADBAND_US:
                        self._pca.write_microseconds(get_channel(i), int(pwm))

            with self._state_lock:
                self._current_pwm = new_pwm

            # ── Precise sleep ─────────────────────────────────────────────────
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    # ── State Publisher (100Hz timer) ─────────────────────────────────────────

    def _publish_state(self) -> None:
        """Publish current joint positions derived from PCA9685 PWM tracking."""
        with self._state_lock:
            pwm_snapshot = list(self._current_pwm)

        msg = Float64MultiArray()
        radians: list[float] = []
        for i in range(NUM_JOINTS):
            rad = pwm_to_rad(pwm_snapshot[i], i)
            # Un-invert before publishing so ROS sees the correct sign convention
            if i in INVERTED_JOINTS:
                rad = -rad
            radians.append(rad)

        msg.data = radians
        self._state_pub.publish(msg)

    # ── Health Publisher (4Hz timer) ─────────────────────────────────────────

    def _publish_health(self) -> None:
        """Publish link health telemetry (same field layout as ESP32 firmware)."""
        now = time.monotonic()
        uptime_s = now - self._start_time

        cmd_age_ms = 0.0
        if self._last_cmd_time is not None:
            cmd_age_ms = (now - self._last_cmd_time) * 1000.0

        # CPU temperature (RPi hardware sensor, replaces WiFi RSSI)
        cpu_temp = -1.0
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                cpu_temp = int(f.read().strip()) / 1000.0
        except OSError:
            pass

        with self._state_lock:
            pwm_snapshot = list(self._current_pwm)

        msg = Float64MultiArray()
        msg.data = [
            uptime_s,                          # [0]  uptime_s
            cmd_age_ms,                        # [1]  cmd_age_ms
            float(self._rx_total),             # [2]  rx_total
            float(self._rx_accepted),          # [3]  rx_accepted
            float(self._drop_bad_size),        # [4]  drop_bad_size
            float(self._drop_invalid),         # [5]  drop_invalid
            float(self._drop_stale_seq),       # [6]  drop_stale_seq
            float(self._timeout_events),       # [7]  timeout_events
            float(self._last_rx_seq),          # [8]  last_rx_seq
            cpu_temp,                          # [9]  cpu_temp_c (was WiFi RSSI)
            1.0,                               # [10] network_ok (always 1 if node runs)
            *[float(p) for p in pwm_snapshot], # [11..24] current_pwm per joint
        ]

        self._health_pub.publish(msg)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._running = False
        self._pca.set_all_off()
        self.get_logger().info('[Bridge] Node destroyed — all PWM outputs zeroed.')
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DexterRpiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
