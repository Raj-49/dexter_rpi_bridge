"""
pca9685_driver.py
=================
Thin wrapper over adafruit-circuitpython-pca9685.

Handles:
  - Initialisation with I2C scan diagnostics
  - write_microseconds(channel, us) — identical API to Adafruit Arduino library
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


class PCA9685Driver:
    """
    Wraps the Adafruit CircuitPython PCA9685 library.
    Provides write_microseconds() matching the Arduino library signature.
    """

    # PCA9685 internal oscillator. Trim this if servos are off by a fixed angle.
    OSCILLATOR_HZ: int = 25_000_000

    def __init__(self, address: int = 0x40, freq_hz: int = 50) -> None:
        self._address = address
        self._freq_hz = freq_hz
        self._period_us: float = 1_000_000.0 / freq_hz   # e.g. 20000µs for 50Hz
        self._pca = None
        self._dry_run = not _HW_AVAILABLE

        if self._dry_run:
            logger.warning('[PCA9685] DRY-RUN: All write_microseconds() calls are no-ops.')
            return

        self._init_hardware()

    # ── Hardware initialisation ───────────────────────────────────────────────

    def _init_hardware(self) -> None:
        """Initialise I2C bus and PCA9685. Logs diagnostic scan."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)

            # I2C scan — diagnostic only, confirms boards are wired correctly
            logger.info('[I2C] Scanning bus...')
            found = []
            while not i2c.try_lock():
                pass
            try:
                addresses = i2c.scan()
                found = addresses
            finally:
                i2c.unlock()

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
            self._pca = PCA9685(i2c, address=self._address)
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

    def write_microseconds(self, channel: int, us: int) -> None:
        """
        Write a PWM pulse width in microseconds to a PCA9685 channel.
        Identical semantics to the Arduino Adafruit_PWMServoDriver::writeMicroseconds().

        Args:
            channel: PCA9685 channel (0-15)
            us:      Pulse width in microseconds (e.g. 1000-2000)
        """
        if self._dry_run or self._pca is None:
            return

        # Convert µs → 16-bit duty cycle
        # duty_cycle = (us / period_us) * 65535
        duty = int((us / self._period_us) * 65535)
        duty = max(0, min(65535, duty))

        try:
            self._pca.channels[channel].duty_cycle = duty
        except Exception as exc:
            logger.error(f'[PCA9685] write ch{channel}={us}µs failed: {exc}')

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
