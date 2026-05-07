"""
pca9685_driver.py
=================
Thin wrapper over adafruit-circuitpython-pca9685.

Handles:
  - Initialisation with I2C scan diagnostics
  - write_microseconds(channel, us) — identical API to Adafruit Arduino library
  - write_microseconds_bulk(channel_us_map) — writes ALL channels in one I2C burst
  - Oscillator frequency calibration
  - Graceful error handling (logs and continues rather than crashing)

Hardware: Raspberry Pi 3 GPIO2 (SDA) / GPIO3 (SCL) → PCA9685 at 0x40
Both PCA9685 boards share the same I2C address (0x40), so a single driver
instance covers all 14 servo channels (0-13).
"""

import logging

logger = logging.getLogger(__name__)

# Try to import hardware libraries. Falls back to a stub for unit-testing
# on a non-RPi machine (e.g., developer laptop).
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    _HW_AVAILABLE = True
except (ImportError, NotImplementedError):
    _HW_AVAILABLE = False
    logger.warning(
        '[PCA9685] adafruit-circuitpython-pca9685 not available. '
        'Running in DRY-RUN mode (no hardware output).'
    )

# PCA9685 register map constants
_MODE1         = 0x00
_LED0_ON_L     = 0x06   # First channel register; each channel = 4 bytes
_ALL_LED_ON_L  = 0xFA


class PCA9685Driver:
    """
    Wraps the Adafruit CircuitPython PCA9685 library.
    Provides write_microseconds() matching the Arduino library signature,
    and write_microseconds_bulk() for writing all channels in one I2C burst.
    """

    # PCA9685 internal oscillator. Trim this if servos are off by a fixed angle.
    OSCILLATOR_HZ: int = 25_000_000

    def __init__(self, address: int = 0x40, freq_hz: int = 50) -> None:
        self._address = address
        self._freq_hz = freq_hz
        self._period_us: float = 1_000_000.0 / freq_hz   # e.g. 20000µs for 50Hz
        self._pca = None
        self._i2c = None       # raw busio.I2C kept for bulk writes
        self._dry_run = not _HW_AVAILABLE

        if self._dry_run:
            logger.warning('[PCA9685] DRY-RUN: All write_microseconds() calls are no-ops.')
            return

        self._init_hardware()

    # ── Hardware initialisation ───────────────────────────────────────────────

    def _init_hardware(self) -> None:
        """Initialise I2C bus and PCA9685. Logs diagnostic scan."""
        try:
            # busio.I2C uses the kernel I2C driver speed — set to 400kHz in
            # /boot/firmware/config.txt: dtparam=i2c_arm_baudrate=400000
            self._i2c = busio.I2C(board.SCL, board.SDA)

            # I2C scan — diagnostic only, confirms boards are wired correctly
            logger.info('[I2C] Scanning bus...')
            found = []
            while not self._i2c.try_lock():
                pass
            try:
                addresses = self._i2c.scan()
                found = addresses
            finally:
                self._i2c.unlock()

            if found:
                for addr in found:
                    logger.info(f'[I2C]  Found device at 0x{addr:02X}')
            else:
                logger.error('[I2C] WARNING: No I2C devices found! Check SDA/SCL wiring.')

            if self._address not in found:
                logger.error(
                    f'[PCA9685] ERROR: 0x{self._address:02X} not on bus! '
                    f'Arm will NOT move. Check VCC, SDA, SCL.'
                )
                self._dry_run = True
                return

            # Initialise PCA9685
            self._pca = PCA9685(self._i2c, address=self._address)
            self._pca.reference_clock_speed = self.OSCILLATOR_HZ
            self._pca.frequency = self._freq_hz

            logger.info(
                f'[PCA9685] OK — 0x{self._address:02X}, '
                f'freq={self._freq_hz}Hz, osc={self.OSCILLATOR_HZ}Hz'
            )

        except Exception as exc:
            logger.error(f'[PCA9685] Init FAILED: {exc}')
            self._dry_run = True

    # ── Public API ────────────────────────────────────────────────────────────

    def _us_to_duty_counts(self, us: int) -> int:
        """Convert µs pulse width → 12-bit PCA9685 OFF count (ON count = 0)."""
        duty_f = (us / self._period_us) * 4096.0
        return max(0, min(4095, int(duty_f)))

    def write_microseconds(self, channel: int, us: int) -> None:
        """
        Write a PWM pulse width in microseconds to a single PCA9685 channel.
        Identical semantics to Adafruit_PWMServoDriver::writeMicroseconds().

        For updating all channels every control cycle, prefer write_microseconds_bulk()
        which performs a single I2C burst and is ~4x faster.
        """
        if self._dry_run or self._pca is None:
            return

        # Convert µs → 16-bit duty cycle (Adafruit CircuitPython uses 16-bit)
        duty = int((us / self._period_us) * 65535)
        duty = max(0, min(65535, duty))

        try:
            self._pca.channels[channel].duty_cycle = duty
        except Exception as exc:
            logger.error(f'[PCA9685] write ch{channel}={us}µs failed: {exc}')

    def write_microseconds_bulk(self, channel_us: dict) -> None:
        """
        Write multiple channels in a SINGLE sequential I2C burst.

        Instead of 14 separate I2C transactions (14 × start+addr+4bytes+stop ≈ 6.3ms
        at 100kHz, 1.6ms at 400kHz), this writes one contiguous block of registers
        starting at LED0_ON_L using PCA9685 auto-increment.

        This reduces per-frame I2C overhead by ~4x.

        Args:
            channel_us: dict mapping channel_index → pulse_width_µs
                        e.g. {0: 1500, 1: 1200, 3: 1800, ...}
                        Channels not in the dict are skipped.
        """
        if self._dry_run or self._pca is None or self._i2c is None:
            return

        if not channel_us:
            return

        # Find contiguous range to minimise bytes sent
        channels_sorted = sorted(channel_us.keys())
        first_ch = channels_sorted[0]
        last_ch  = channels_sorted[-1]

        # Build register payload: 4 bytes per channel (ON_L, ON_H, OFF_L, OFF_H)
        # ON count = 0 (start of period), OFF count = computed from µs
        n_channels = last_ch - first_ch + 1
        payload = bytearray(4 * n_channels)

        for ch_idx in range(n_channels):
            ch = first_ch + ch_idx
            us = channel_us.get(ch, None)
            if us is None:
                # Gap channel — leave registers at 0 (channel already set)
                continue
            off_count = self._us_to_duty_counts(us)
            base = ch_idx * 4
            payload[base + 0] = 0            # ON_L  (ON always at 0)
            payload[base + 1] = 0            # ON_H
            payload[base + 2] = off_count & 0xFF         # OFF_L
            payload[base + 3] = (off_count >> 8) & 0x0F  # OFF_H

        # Write: register address = LED0_ON_L + first_ch * 4
        reg_start = _LED0_ON_L + first_ch * 4

        try:
            while not self._i2c.try_lock():
                pass
            try:
                # writeto() with auto-increment: send reg addr + payload in one call
                self._i2c.writeto(self._address, bytes([reg_start]) + payload)
            finally:
                self._i2c.unlock()
        except Exception as exc:
            logger.error(f'[PCA9685] bulk_write failed: {exc}')
            # Fallback: individual writes
            for ch, us in channel_us.items():
                self.write_microseconds(ch, us)

    def set_all_off(self) -> None:
        """Zero all channels — emergency stop."""
        if self._dry_run or self._pca is None:
            return
        for ch in range(16):
            try:
                self._pca.channels[ch].duty_cycle = 0
            except Exception:
                pass

    @property
    def is_ready(self) -> bool:
        """True if hardware is initialised and responding."""
        return not self._dry_run and self._pca is not None
