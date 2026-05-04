# dexter-rpi-bridge

Raspberry Pi hardware bridge for the **Dexter dual-arm robot**.

Replaces the ESP32 + micro-ROS firmware entirely. Runs on **Raspberry Pi 3 Model B**,
connects to PCA9685 servo driver boards via I2C, and communicates with the main
ROS 2 stack on the laptop over WiFi using native DDS (no micro-ROS agent needed).

---

## Architecture

```
Laptop (ROS 2 Jazzy)                   Raspberry Pi 3
────────────────────                   ──────────────────────────────
ros2_control / MoveIt                  dexter_rpi_bridge (this package)
  DexterHardwareInterface                hardware_node.py
    publishes /rpi/joint_commands  ──►    subscribes /rpi/joint_commands
    subscribes /rpi/joint_states   ◄──    publishes  /rpi/joint_states
    subscribes /rpi/link_health    ◄──    publishes  /rpi/link_health
                                              │
                                         PCA9685 (0x40) via I2C
                                              │
                                         14 servo motors
```

---

## Hardware Wiring

| RPi Pin | RPi Signal | → | PCA9685 |
|---------|-----------|---|---------|
| Pin 1   | 3.3V      | → | VCC     |
| Pin 3   | GPIO2/SDA | → | SDA     |
| Pin 5   | GPIO3/SCL | → | SCL     |
| Pin 6   | GND       | → | GND     |
| —       | External PSU 5-6V | → | V+ |
| —       | External PSU GND  | → | GND (common) |

**Two PCA9685 boards share the same I2C address (0x40).**
- Board 1: channels 0-5 (left/right arm, RDS3235 35kg heavy servos)
- Board 2: channels 6-13 (wrist + grippers, RKI-1206 15kg light servos)

---

## Quick Start (on Raspberry Pi)

```bash
# 1. One-shot install (run via SSH on fresh RPi OS Lite 64-bit)
bash <(curl -fsSL https://raw.githubusercontent.com/Raj-49/dexter-rpi-bridge/main/scripts/install_rpi.sh)

# 2. After reboot, verify I2C
python3 scripts/i2c_test.py

# 3. Start the bridge
sudo systemctl start dexter-rpi-bridge

# 4. Watch logs
journalctl -u dexter-rpi-bridge -f
```

---

## Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/rpi/joint_commands` | `Float64MultiArray` (15 elements) | Laptop → RPi | 25 Hz |
| `/rpi/joint_states`   | `Float64MultiArray` (14 elements) | RPi → Laptop | 100 Hz |
| `/rpi/link_health`    | `Float64MultiArray` (25 elements) | RPi → Laptop | 4 Hz |

### joint_commands format
`[j0, j1, ..., j13, seq_id]` — 14 joint angles (radians) + monotonic sequence counter

### link_health format
`[uptime_s, cmd_age_ms, rx_total, rx_accepted, drop_bad_size, drop_invalid,
  drop_stale_seq, timeout_events, last_rx_seq, cpu_temp_c, network_ok,
  pwm0..pwm13]`

---

## Updating Code

```bash
# SSH into RPi
ssh pi@dexter-rpi.local

# Pull latest, rebuild, restart
cd ~/dexter_rpi_ws/src/dexter-rpi-bridge && git pull
cd ~/dexter_rpi_ws && colcon build --symlink-install
sudo systemctl restart dexter-rpi-bridge
```

---

## Joint Index Mapping

| Index | Name | Joint | PCA ch | Servo | Inverted |
|-------|------|-------|--------|-------|---------|
| 0 | jl1 | Left Base | 0 | RDS3235 | No |
| 1 | jl2 | Left Shoulder | 1 | RDS3235 | No |
| 2 | jl3 | Left Elbow | 2 | RDS3235 | **Yes** |
| 3 | jl4 | Left Wrist Pitch | 6 | RKI-1206 | No |
| 4 | jl5 | Left Wrist Roll | 7 | RKI-1206 | No |
| 5 | jl6 | Left End | 8 | RKI-1206 | **Yes** |
| 6 | jl7 | Left Gripper | 9 | RKI-1206 | No |
| 7 | jr1 | Right Base | 3 | RDS3235 | No |
| 8 | jr2 | Right Shoulder | 4 | RDS3235 | **Yes** |
| 9 | jr3 | Right Elbow | 5 | RDS3235 | **Yes** |
| 10 | jr4 | Right Wrist Pitch | 10 | RKI-1206 | **Yes** |
| 11 | jr5 | Right Wrist Roll | 11 | RKI-1206 | **Yes** |
| 12 | jr6 | Right End | 12 | RKI-1206 | No |
| 13 | jr7 | Right Gripper | 13 | RKI-1206 | No |
