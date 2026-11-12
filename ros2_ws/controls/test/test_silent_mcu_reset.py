"""
Tests for the silent-MCU watchdog.

A port that is open but carries nothing gets the ESP32 reset over the USB
modem lines, into its normal boot (not the bootloader).

    python3 -m pytest ros2_ws/controls/test/test_silent_mcu_reset.py -v
"""

from controls.mcu_bridge import MCU_SILENT_RESET_S, mcu_is_silent, reset_mcu


class FakePort:
    """Records the (dtr, rts) levels set on it, in order."""

    def __init__(self):
        self.levels = []
        self._dtr = True
        self._rts = True

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        self.levels.append((self._dtr, self._rts))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value
        self.levels.append((self._dtr, self._rts))


def test_silence_threshold():
    assert MCU_SILENT_RESET_S > 0
    assert not mcu_is_silent(10.0, 9.0, 4.0)
    assert mcu_is_silent(13.0, 9.0, 4.0)
    assert mcu_is_silent(14.0, 9.0, 4.0)


def test_watchdog_disabled_or_unarmed():
    assert not mcu_is_silent(100.0, 0.0, 0.0)    # 0 disables it
    assert not mcu_is_silent(100.0, None, 4.0)   # never connected


def test_reset_pulse_boots_from_flash():
    port = FakePort()
    slept = []
    reset_mcu(port, pulse_s=0.1, sleep=slept.append)
    assert slept == [0.1]
    # DTR stays low throughout (DTR high leaving reset = ROM bootloader),
    # RTS goes high (chip held in reset) and is released low again
    assert all(dtr is False for dtr, _ in port.levels)
    assert [rts for _, rts in port.levels][-2:] == [True, False]
    assert (port.dtr, port.rts) == (False, False)
