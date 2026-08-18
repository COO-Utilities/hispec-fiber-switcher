"""Low-level Python driver for the fiber-optic port switcher PLC.

The switcher is controlled by a Mitsubishi MELSEC PLC speaking the MC
protocol (3E frame, ASCII) over TCP, with the PLC acting as the TCP server
(the host running this driver is the TCP client).

Two independent ports are exposed:
- Port A: a 13-position rotary/vertical selector (positions 1-13).
- Port B: a 5-position movable-side selector (positions 1-5).

A third axis (the connector insertion/removal axis) inserts or retracts the
fiber at whichever position Ports A/B are currently pointed at.

Protocol notes (see README.md for more detail):
- Port target positions are transmitted as BCD-style words
- The PLC's "W" link-register addresses are hex under the hood
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Tuple

from pymcprotocol import Type3E

from hardware_device_base.hardware_motion_base import HardwareMotionBase

# Connection defaults
DEFAULT_PORT = 9001

# PLC device addresses
_STATUS_HEAD_DEVICE = "W0"  # 14 words, see read_status()
_STATUS_WORD_COUNT = 14
_TARGET_POSITION_DEVICE = "W0E"  # 2 words: [port A code, port B code]
_CONTROL_WORD_DEVICE = "W0C"  # 1 word
_AIR_PURGE_DEVICE = "M381"  # 1 bit: 1=open, 0=close
_REARMOST_DEVICE = "D200"  # 1 word, fixed value 5

# Control words for _CONTROL_WORD_DEVICE
CONTROL_WORD_HOME = 0x0001
CONTROL_WORD_STOP = 0x0002
CONTROL_WORD_RETRACT_FIBER = 0x0004
CONTROL_WORD_INSERT_FIBER = 0x0008
CONTROL_WORD_MOVE_TO_TARGET = 0x0010
CONTROL_WORD_CLEAN_INSIDE = 0x0120
CONTROL_WORD_CLEAN_OUTSIDE = 0x0220
CONTROL_WORD_CLEAN_BOTH = 0x0320

_CLEAN_MODES = {
    "inside": CONTROL_WORD_CLEAN_INSIDE,
    "outside": CONTROL_WORD_CLEAN_OUTSIDE,
    "both": CONTROL_WORD_CLEAN_BOTH,
}

AXIS_REGISTER_RETRACT_DISTANCE = "1228"
AXIS_REGISTER_CAMERA_INSERTION_POSITION = "1250"
AXIS_REGISTER_NONCAMERA_INSERTION_POSITION = "1258"

CLEAN_POSITION_PORT_A = 10
CLEAN_POSITION_PORT_B = 2

# Valid position indexes
PORT_A_POSITIONS: Tuple[int, ...] = tuple(range(1, 14))  # 1-13
PORT_B_POSITIONS: Tuple[int, ...] = tuple(range(1, 6))  # 1-5

# Insertion/removal axis state
INSERTION_STATE_REMOVED = 1
INSERTION_STATE_INSERTED = 3

# Known alarm/message bits
ALM_M01_ORIGIN_RETURN_TIMEOUT = 0x1000
MSG_AUTOMATIC_OPERATION_AVAILABLE = 0x0004


@dataclass
class SwitcherStatus:

    port_a_target: int
    port_b_target: int
    in_position: bool
    insertion_state: int
    horizontal_axis_position: int
    insertion_axis_position: int
    alarms_0_15: int
    alarms_16_31: int
    messages_0_15: int
    messages_16_31: int


def _position_to_bcd_word(position: int) -> int:
    """Encode a 1-based port position index as its BCD-style register word.
    """
    return int(f"01{position:02d}", 16)


def _target_code_to_position(code: int) -> int:
    """Convert a target code back to a position.
    """
    return code - 100


def _position_to_target_code(position: int) -> int:
    """Inverse of `_target_code_to_position`."""
    return position + 100


class FiberSwitcher(HardwareMotionBase):
    """Control the fiber-optic port switcher PLC over TCP.

    Example:
        >>> switcher = FiberSwitcher()
        >>> switcher.connect("192.168.0.183", 9001)
        >>> switcher.home()
        >>> switcher.set_target_positions(port_a=5, port_b=2)
        >>> switcher.insert_fiber()
        >>> switcher.disconnect()
    """

    def __init__(
        self,
        log: bool = True,
        logfile: str = __name__.rsplit(".", 1)[-1],
        read_timeout: float = 2.0,
    ) -> None:
        super().__init__(log, logfile)
        self._client = Type3E(plctype="Q")
        self._client.setaccessopt(commtype="ascii")
        self._client.timer = 0
        self.read_timeout = read_timeout
        self.host: str | None = None
        self.port: int | None = None
        self._homed = False
        self._last_reply: Any | None = None

        # Set after clean() to the known post-clean position, since the PLC
        # doesn't update its own target registers for that move.
        self._position_override: tuple[int, int] | None = None

    def connect(self, host: str, port: int = DEFAULT_PORT) -> None:  # pylint: disable=W0221
        """Open a TCP connection to the PLC.

        :param str host: PLC IP address (e.g. "192.168.0.183").
        :param int port: PLC port number (9001 for the first unit).
        """
        if self.is_connected():
            self.disconnect()

        self.report_info(f"Connecting to fiber switcher at {host}:{port}...")
        self._client.soc_timeout = self.read_timeout
        try:
            self._client.connect(host, port)
        except OSError as exc:
            self.report_error(f"Could not connect to {host}:{port}: {exc}")
            raise ConnectionError(f"Could not connect to {host}:{port}") from exc

        self.host = host
        self.port = port
        self._set_connected(True)
        self.report_info("Fiber switcher connected")

    def disconnect(self) -> None:
        """Close the TCP connection to the PLC."""
        if self.is_connected():
            self._client.close()
        self._set_connected(False)
        self.report_info("Fiber switcher disconnected")

    def _require_connection(self) -> None:
        if not self.is_connected():
            raise RuntimeError("Fiber switcher is not connected")

    def _send_command(self, method: str, *args, **kwargs) -> bool:  # pylint: disable=W0221
        """Invoke a `pymcprotocol.Type3E` method by name and cache its result.
        """
        self._require_connection()
        with self.lock:
            self._last_reply = getattr(self._client, method)(*args, **kwargs)
        return True

    def _read_reply(self) -> Any:
        """Return the result cached by the most recent `_send_command` call."""
        return self._last_reply

    def _call(self, method: str, *args, **kwargs) -> Any:
        """Send a `pymcprotocol.Type3E` method call and return its result."""
        self._send_command(method, *args, **kwargs)
        return self._read_reply()

    def read_status(self) -> SwitcherStatus:
        """Read and parse the PLC's full device-status reply."""
        words = [value & 0xFFFF for value in self._call("batchread_wordunits", _STATUS_HEAD_DEVICE, _STATUS_WORD_COUNT)]
        if self._position_override is not None:
            port_a_target = _position_to_target_code(self._position_override[0])
            port_b_target = _position_to_target_code(self._position_override[1])
        else:
            port_a_target = int(f"{words[0]:04X}")
            port_b_target = int(f"{words[1]:04X}")
        return SwitcherStatus(
            port_a_target=port_a_target,
            port_b_target=port_b_target,
            in_position=words[2] == 1,
            insertion_state=words[3],
            horizontal_axis_position=words[4] | (words[5] << 16),
            insertion_axis_position=words[6] | (words[7] << 16),
            # words[8:10] are reserved, per the vendor document.
            alarms_0_15=words[10],
            alarms_16_31=words[11],
            messages_0_15=words[12],
            messages_16_31=words[13],
        )

    def home(
        self,
        wait: bool = True,
        timeout: float = 60.0,
        poll_interval: float = 0.5,
    ) -> bool:
        """Start return-to-origin and, by default, wait for it to finish.
        """
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [CONTROL_WORD_HOME], [], [])
        self.report_info("Return-to-origin command sent")
        self._homed = False
        self._position_override = None
        if not wait:
            return True

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.read_status()
            if status.alarms_0_15 & ALM_M01_ORIGIN_RETURN_TIMEOUT:
                message = "Fiber switcher reported an origin-return timeout alarm"
                self.report_error(message)
                raise RuntimeError(message)
            if status.in_position:
                self._homed = True
                self.report_info("Return-to-origin complete")
                return True
            time.sleep(poll_interval)

        self.report_warning("Timed out waiting for return-to-origin to complete")
        return False

    def is_homed(self) -> bool:
        return self._homed

    def is_loop_closed(self) -> bool:
        """Return the PLC's target/current position-match flag.
        The switcher's positioning loop is always closed.
        """
        return self.read_status().in_position

    def close_loop(self) -> bool:
        """This device has no selectable open/closed-loop mode."""
        self.report_warning("close_loop() is a no-op; this device has no open-loop mode")
        return self.is_loop_closed()

    def get_pos(self, axis: str = "A") -> int:  # pylint: disable=W0221
        """Return the current target position index (1-based) for ``axis``.
        :param str axis: "A" or "B".
        """
        axis = axis.upper()
        if axis not in ("A", "B"):
            raise ValueError(f"Unknown axis {axis!r}; expected 'A' or 'B'")
        if self._position_override is not None:
            port_a, port_b = self._position_override
        else:
            status = self.read_status()
            port_a = _target_code_to_position(status.port_a_target)
            port_b = _target_code_to_position(status.port_b_target)
        return port_a if axis == "A" else port_b

    def get_limits(self) -> Dict[str, Tuple[int, int]]:
        """Return the documented position-index range for each port."""
        return {
            "A": (PORT_A_POSITIONS[0], PORT_A_POSITIONS[-1]),
            "B": (PORT_B_POSITIONS[0], PORT_B_POSITIONS[-1]),
        }

    def set_target_positions(self, port_a: int, port_b: int) -> bool:
        """Set both port target positions in a single command.

        :param int port_a: Port A position index, 1-13.
        :param int port_b: Port B position index, 1-5.
        """
        if port_a not in PORT_A_POSITIONS:
            raise ValueError(
                f"Port A position {port_a} out of range "
                f"{PORT_A_POSITIONS[0]}-{PORT_A_POSITIONS[-1]}"
            )
        if port_b not in PORT_B_POSITIONS:
            raise ValueError(
                f"Port B position {port_b} out of range "
                f"{PORT_B_POSITIONS[0]}-{PORT_B_POSITIONS[-1]}"
            )
        values = [_position_to_bcd_word(port_a), _position_to_bcd_word(port_b)]
        self._call("batchwrite_wordunits", _TARGET_POSITION_DEVICE, values)
        self._position_override = None
        return True

    def set_pos(self, axis: str, position: int) -> bool:  # pylint: disable=W0221
        """Set the target position index (1-based) for one port.

        The PLC only exposes a single command that sets both ports'
        targets together, so this reads the other port's current target
        first and resends it unchanged.

        :param str axis: "A" or "B".
        :param int position: 1-13 for Port A, 1-5 for Port B.
        """
        axis = axis.upper()
        if axis not in ("A", "B"):
            raise ValueError(f"Unknown axis {axis!r}; expected 'A' or 'B'")
        port_a = position if axis == "A" else self.get_pos("A")
        port_b = position if axis == "B" else self.get_pos("B")
        return self.set_target_positions(port_a, port_b)

    def stop(self) -> bool:
        """Stop motor motion in progress."""
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [CONTROL_WORD_STOP], [], [])
        return True

    def retract_fiber(self) -> bool:
        """Retract the fiber at the current position.
        Only works in automatic operation mode.
        """
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [CONTROL_WORD_RETRACT_FIBER], [], [])
        return True

    def insert_fiber(self) -> bool:
        """Insert the fiber at the current target position.
        Only works in automatic operation mode.
        """
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [CONTROL_WORD_INSERT_FIBER], [], [])
        return True

    def move_to_target(self) -> bool:
        """Move to the current target fiber position without inserting it."""
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [CONTROL_WORD_MOVE_TO_TARGET], [], [])
        return True

    def clean(self, mode: str = "both") -> bool:
        """Start a cleaning cycle.

        :param str mode: One of "inside", "outside", or "both".
        """
        if mode not in _CLEAN_MODES:
            raise ValueError(f"Unknown clean mode {mode!r}; expected one of {sorted(_CLEAN_MODES)}")
        self._call("randomwrite", [_CONTROL_WORD_DEVICE], [_CLEAN_MODES[mode]], [], [])
        self._position_override = (CLEAN_POSITION_PORT_A, CLEAN_POSITION_PORT_B)
        return True

    def move_to_rearmost(self) -> bool:
        """Move the insertion/removal axis to the rearmost traversing
        position. Only valid once homing has completed."""
        self._call("batchwrite_wordunits", _REARMOST_DEVICE, [5])
        return True

    def open_air_purge(self) -> bool:
        """Open the air purge valve."""
        self._call("batchwrite_bitunits", _AIR_PURGE_DEVICE, [1])
        return True

    def close_air_purge(self) -> bool:
        """Close the air purge valve."""
        self._call("batchwrite_bitunits", _AIR_PURGE_DEVICE, [0])
        return True

    def write_axis_register(self, register: str, value_mm: float) -> bool:
        """Write a 0.01 mm-resolution axis-setting register.

        :param str register: The 4-digit D-device register number.
        :param float value_mm: Distance/position in millimeters.

        .. warning::
            This writes directly to PLC memory. Values outside the
            device's operational range may cause malfunction or damage.
        """
        if not (register.isdigit() and len(register) == 4):
            raise ValueError(f"register must be a 4-digit numeric string, got {register!r}")
        raw = round(value_mm * 100)
        if not 0 <= raw <= 0xFFFFFFFF:
            raise ValueError(f"value_mm {value_mm} is out of the encodable range")
        values = [raw & 0xFFFF, (raw >> 16) & 0xFFFF]
        self._call("batchwrite_wordunits", f"D{register}", values)
        return True

    def read_axis_register(self, register: str) -> float:
        """Read a 0.01 mm-resolution axis-setting register.

        :param str register: The 4-digit D-device register number.
        :return: The register's value in millimeters.
        """
        if not (register.isdigit() and len(register) == 4):
            raise ValueError(f"register must be a 4-digit numeric string, got {register!r}")
        low, high = (
            value & 0xFFFF for value in self._call("batchread_wordunits", f"D{register}", 2)
        )
        return (low | (high << 16)) / 100

    def set_retract_distance_mm(self, value_mm: float) -> bool:
        """Set the insertion/removal axis's retract-back distance."""
        return self.write_axis_register(AXIS_REGISTER_RETRACT_DISTANCE, value_mm)

    def get_retract_distance_mm(self) -> float:
        """Get the insertion/removal axis's retract-back distance."""
        return self.read_axis_register(AXIS_REGISTER_RETRACT_DISTANCE)

    def set_camera_insertion_position_mm(self, value_mm: float) -> bool:
        """Set the insertion position used for camera observation ports."""
        return self.write_axis_register(AXIS_REGISTER_CAMERA_INSERTION_POSITION, value_mm)

    def get_camera_insertion_position_mm(self) -> float:
        """Get the insertion position used for camera observation ports."""
        return self.read_axis_register(AXIS_REGISTER_CAMERA_INSERTION_POSITION)

    def set_noncamera_insertion_position_mm(self, value_mm: float) -> bool:
        """Set the insertion position used for non-camera observation ports."""
        return self.write_axis_register(AXIS_REGISTER_NONCAMERA_INSERTION_POSITION, value_mm)

    def get_noncamera_insertion_position_mm(self) -> float:
        """Get the insertion position used for non-camera observation ports."""
        return self.read_axis_register(AXIS_REGISTER_NONCAMERA_INSERTION_POSITION)

    def __enter__(self) -> "FiberSwitcher":
        return self

    def __exit__(self, *_: object) -> None:
        self.disconnect()
