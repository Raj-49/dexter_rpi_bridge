#!/usr/bin/env python3
"""
hardware_node.py — Dexter RPi Bridge (roslibpy version)
========================================================
NO ROS 2 INSTALLATION NEEDED ON THE RASPBERRY PI.

This script connects to the rosbridge_server running on the laptop
via WebSocket, subscribes to /rpi/joint_commands, drives the PCA9685
via I2C, and publishes /rpi/joint_states.

Requirements (pip install only):
    pip3 install roslibpy adafruit-circuitpython-pca9685 adafruit-blinka

Usage:
    python3 hardware_node.py --host 192.168.x.x
    python3 hardware_node.py --host 192.168.x.x --port 9090

The laptop IP is the machine running rosbridge_server.
"""

import argparse
import math
import os
import sys
import threading
import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger('dexter_bridge')

# ── Local imports (same directory) ───────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)

from servo_config import (
    SERVO_CONFIG, INVERTED_JOINTS, SOFT_LIMIT_RAD,
    SERVO_DEADBAND_US, COMMAND_TIMEOUT_S, CONTROL_HZ,
    MAX_VELOCITY_US_PER_S, MAX_ACCEL_US_PER_S2,
    NUM_JOINTS, rad_to_pwm, pwm_to_rad, get_init_pwm, get_channel, get_name,
)
from pca9685_driver import PCA9685Driver
from motion_profile import MotionProfile

try:
    import roslibpy
except ImportError:
    log.error("roslibpy not installed. Run: pip3 install roslibpy")
    sys.exit(1)


class DexterRpiBridge:
    """Hardware bridge: rosbridge WebSocket → PCA9685 → servos."""

    def __init__(self, host: str, port: int = 9090) -> None:
        self._host = host
        self._port = port

        # ── PCA9685 ───────────────────────────────────────────────────────────
        self._pca = PCA9685Driver(address=0x40, freq_hz=50)
        if not self._pca.is_ready:
            log.warning('PCA9685 not ready — DRY-RUN mode (no servo output)')

        # ── Motion profiles (one per joint) ──────────────────────────────────
        self._profiles = [
            MotionProfile(
                init_pwm=get_init_pwm(i),
                max_velocity_us_s=MAX_VELOCITY_US_PER_S,
                max_accel_us_s2=MAX_ACCEL_US_PER_S2,
                deadband_us=SERVO_DEADBAND_US,
            )
            for i in range(NUM_JOINTS)
        ]

        # ── Shared state ──────────────────────────────────────────────────────
        self._lock = threading.Lock()
        # Set to -1.0 so the deadband check forces a write on the very first loop
        self._current_pwm = [-1.0 for _ in range(NUM_JOINTS)]
        # Engage immediately on boot to hold home position
        self._engaged = [True] * NUM_JOINTS

        # ── Latest command (thread-safe single slot) ──────────────────────────
        self._cmd_lock = threading.Lock()
        self._latest_cmd = None

        # ── Telemetry ─────────────────────────────────────────────────────────
        self._rx_total       = 0
        self._rx_accepted    = 0
        self._drop_bad_size  = 0
        self._drop_invalid   = 0
        self._drop_stale_seq = 0
        self._timeout_events = 0
        self._last_rx_seq    = -1
        self._seq_init       = False
        self._last_cmd_time  = None
        self._cmd_stale      = False
        self._start_time     = time.monotonic()

        # ── roslib client + topics ────────────────────────────────────────────
        self._client = roslibpy.Ros(host=host, port=port)
        self._client.on_ready(self._on_connected)
        self._client.on('close', self._on_close)
        self._client.on('error', lambda e: log.error(f'rosbridge error: {e}'))

        self._cmd_topic = roslibpy.Topic(
            self._client, '/rpi/joint_commands',
            'std_msgs/msg/Float64MultiArray')
        self._state_topic = roslibpy.Topic(
            self._client, '/rpi/joint_states',
            'std_msgs/msg/Float64MultiArray')
        self._health_topic = roslibpy.Topic(
            self._client, '/rpi/link_health',
            'std_msgs/msg/Float64MultiArray')

        # ── Background threads ────────────────────────────────────────────────
        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='dexter_ctrl')
        self._state_thread = threading.Thread(
            target=self._state_loop, daemon=True, name='dexter_state')
        self._health_thread = threading.Thread(
            target=self._health_loop, daemon=True, name='dexter_health')

    # ── Connection callback ───────────────────────────────────────────────────

    def _on_connected(self):
        log.info(f'✓ Connected to rosbridge at {self._host}:{self._port}')
        self._cmd_topic.subscribe(self._command_callback)
        time.sleep(0.1) # allow time for websocket to register sub
        log.info('  Subscribed to /rpi/joint_commands')
        log.info('  Publishing  /rpi/joint_states  @ 100Hz')
        log.info('  Publishing  /rpi/link_health   @   4Hz')

    def _on_close(self):
        log.error('rosbridge connection closed! Forcing clean restart via systemd...')
        os._exit(1)

    # ── ROS Command Callback ──────────────────────────────────────────────────

    def _command_callback(self, message: dict) -> None:
        """Receive joint command from rosbridge and push to control thread."""
        self._rx_total += 1
        data = message.get('data', [])

        if len(data) < NUM_JOINTS:
            self._drop_bad_size += 1
            return

        # Sequence ID validation
        if len(data) >= NUM_JOINTS + 1:
            seq_d = data[NUM_JOINTS]
            if not math.isfinite(seq_d) or seq_d < 0:
                self._drop_invalid += 1
                return
            seq = int(seq_d)
            if self._seq_init and seq <= self._last_rx_seq:
                self._drop_stale_seq += 1
                return
            self._seq_init = True
            self._last_rx_seq = seq

        for val in data[:NUM_JOINTS]:
            if not math.isfinite(val):
                self._drop_invalid += 1
                return

        self._rx_accepted += 1
        self._last_cmd_time = time.monotonic()
        self._cmd_stale = False

        with self._cmd_lock:
            self._latest_cmd = list(data[:NUM_JOINTS])

    # ── Control Loop (100Hz) ──────────────────────────────────────────────────

    def _control_loop(self) -> None:
        dt = 1.0 / CONTROL_HZ
        next_tick = time.monotonic()

        while self._running:
            next_tick += dt
            now = time.monotonic()

            # Command timeout
            if self._last_cmd_time and not self._cmd_stale:
                if (now - self._last_cmd_time) > COMMAND_TIMEOUT_S:
                    self._timeout_events += 1
                    self._cmd_stale = True
                    for i in range(NUM_JOINTS):
                        self._profiles[i].set_target(int(self._profiles[i].current_pwm))

            with self._cmd_lock:
                cmd = self._latest_cmd

            if cmd is not None and not self._cmd_stale:
                for i in range(NUM_JOINTS):
                    rad = max(-SOFT_LIMIT_RAD, min(SOFT_LIMIT_RAD, float(cmd[i])))
                    if i in INVERTED_JOINTS:
                        rad = -rad
                    target_pwm = rad_to_pwm(rad, i)

                    if not self._engaged[i]:
                        self._profiles[i].snap_to(target_pwm)
                        self._engaged[i] = True
                        continue
                    self._profiles[i].set_target(target_pwm)

            new_pwm = []
            for i in range(NUM_JOINTS):
                pwm = float(self._profiles[i].step(dt))
                new_pwm.append(pwm)
                if self._engaged[i]:
                    prev = self._current_pwm[i]
                    if abs(pwm - prev) > SERVO_DEADBAND_US:
                        self._pca.write_microseconds(get_channel(i), int(pwm))

            with self._lock:
                self._current_pwm = new_pwm

            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

    # ── State Publisher (100Hz) ───────────────────────────────────────────────

    def _state_loop(self) -> None:
        period = 1.0 / 100
        while self._running:
            start = time.monotonic()
            if self._client.is_connected:
                with self._lock:
                    pwm_snap = list(self._current_pwm)
                radians = []
                for i in range(NUM_JOINTS):
                    rad = pwm_to_rad(pwm_snap[i], i)
                    if i in INVERTED_JOINTS:
                        rad = -rad
                    radians.append(rad)
                self._state_topic.publish(roslibpy.Message({'data': radians}))
            elapsed = time.monotonic() - start
            time.sleep(max(0, period - elapsed))

    # ── Health Publisher (4Hz) ────────────────────────────────────────────────

    def _health_loop(self) -> None:
        period = 1.0 / 4
        while self._running:
            start = time.monotonic()
            if self._client.is_connected:
                now = time.monotonic()
                cmd_age_ms = (now - self._last_cmd_time) * 1000 if self._last_cmd_time else 0.0

                cpu_temp = -1.0
                try:
                    with open('/sys/class/thermal/thermal_zone0/temp') as f:
                        cpu_temp = int(f.read().strip()) / 1000.0
                except OSError:
                    pass

                with self._lock:
                    pwm_snap = list(self._current_pwm)

                payload = [
                    now - self._start_time,        # [0]  uptime_s
                    cmd_age_ms,                    # [1]  cmd_age_ms
                    float(self._rx_total),         # [2]  rx_total
                    float(self._rx_accepted),      # [3]  rx_accepted
                    float(self._drop_bad_size),    # [4]
                    float(self._drop_invalid),     # [5]
                    float(self._drop_stale_seq),   # [6]
                    float(self._timeout_events),   # [7]
                    float(self._last_rx_seq),      # [8]
                    cpu_temp,                      # [9]  RPi CPU temp (°C)
                    1.0,                           # [10] network_ok
                    *[float(p) for p in pwm_snap], # [11..24] raw PWM
                ]
                self._health_topic.publish(roslibpy.Message({'data': payload}))
            elapsed = time.monotonic() - start
            time.sleep(max(0, period - elapsed))

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        log.info(f'Connecting to rosbridge at ws://{self._host}:{self._port}')
        log.info(f'PCA9685: {"READY" if self._pca.is_ready else "DRY-RUN"}')
        log.info(f'Joints: {NUM_JOINTS}  Control: {CONTROL_HZ}Hz')
        self._control_thread.start()
        self._state_thread.start()
        self._health_thread.start()
        
        while self._running:
            try:
                self._client.run(timeout=86400)   # blocks until Ctrl+C, wait up to 24h for initial connection
            except Exception as e:
                log.error(f"roslibpy run exception: {e}")
                time.sleep(2)
                if not self._client.is_connected:
                    log.error("Client disconnected, forcing clean restart...")
                    os._exit(1)

    def stop(self) -> None:
        self._running = False
        self._pca.set_all_off()
        self._client.terminate()
        log.info('Bridge stopped — all PWM outputs zeroed.')


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    # IP can be passed as argument, env var, or read from config file
    parser = argparse.ArgumentParser(description='Dexter RPi Hardware Bridge')
    parser.add_argument('--host', default=os.environ.get('LAPTOP_IP', ''),
                        help='Laptop IP running rosbridge_server')
    parser.add_argument('--port', type=int, default=9090,
                        help='rosbridge WebSocket port (default: 9090)')
    args = parser.parse_args()

    # Try reading from config file if no host given
    if not args.host:
        cfg = os.path.expanduser('~/dexter_bridge.env')
        if os.path.exists(cfg):
            for line in open(cfg):
                if line.startswith('LAPTOP_IP='):
                    args.host = line.strip().split('=', 1)[1]
                    break

    if not args.host:
        log.error('No laptop IP provided.')
        log.error('Fix: echo "LAPTOP_IP=192.168.x.x" > ~/dexter_bridge.env')
        log.error('  or: python3 hardware_node.py --host 192.168.x.x')
        sys.exit(1)

    bridge = DexterRpiBridge(host=args.host, port=args.port)
    try:
        bridge.start()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
