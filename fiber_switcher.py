"""Low-level Python driver for the fiber-optic port switcher PLC.

The switcher is controlled by a Mitsubishi PLC that acts as a TCP
*server* (the host running this driver is the TCP *client*). Commands are
fixed-format ASCII hex strings; see the vendor communication-protocol
document for the authoritative reference.

Two independent ports are exposed:

* **Port A** -- a 13-position rotary/vertical selector (positions 1-13).
* **Port B** -- a 5-position movable-side selector (positions 1-5).

A third axis (the connector insertion/removal axis) inserts or retracts the
fiber at whichever position Ports A/B are currently pointed at.
"""

from __future__ import annotations

import socket
import time
from dataclasses import dataclass
from typing import Dict, Tuple

from hardware_device_base.hardware_motion_base import HardwareMotionBase

# Connection defaults
DEFAULT_PORT = 9001


# Frame layout
# Every request/reply is an ASCII string shaped like:
#   subheader(4) + network_no(2) + pc_no(2) + io_no(4) + station_no(2)
#   + data_length(4, hex) + <data_length hex-decoded characters follow>
# The first four fields above (18 characters) are a fixed-size header; the
# data-length field tells you exactly how many characters follow, so framing
# a reply never requires guessing at a terminator. This was confirmed against
# every worked example in the vendor document (header + data_length ==
# len(whole string) in each case).
_HEADER_LEN = 18
_ACK_OK = "0000"

# Read device status
_READ_STATUS_CMD = "500000FF03FF000018000004010000W*000000000E"

# Set target position. Append port A code + port B code (4 ASCII
# digits each, see _position_to_code)
_SET_TARGET_PREFIX = "500000FF03FF000020000014010000W*0000140002"

# Motion control words. Append a 4-hex-digit control code
# (see the CONTROL_WORD_* constants below)
_CONTROL_WORD_PREFIX = "500000FF03FF00001C0000140200000100W*000012"

# Air purge valve control (2026/06/11 addendum). 
# Append "1" to open, "0" to close
_AIR_PURGE_PREFIX = "500000FF03FF000019000014010001M*0003810001"

# Move insertion/removal axis to rearmost traversing position. Fixed
# command, no placeholders. Only valid after homing
_REARMOST_CMD = "500000FF03FF00001C000014010000D*00020000010005"

# Write a 0.01 mm-resolution axis-setting register. Append a
# 32-bit little-word-swapped hex value (see _encode_32bit_swapped)
_AXIS_REGISTER_PREFIX = "500000FF03FF000020000014010000D*00{register}0002"

# Axis-setting registers
AXIS_REGISTER_RETRACT_DISTANCE = "1228"
AXIS_REGISTER_CAMERA_INSERTION_POSITION = "1250"
AXIS_REGISTER_NONCAMERA_INSERTION_POSITION = "1258"

# Control words for _CONTROL_WORD_PREFIX
CONTROL_WORD_HOME = "0001"
CONTROL_WORD_STOP = "0002"
CONTROL_WORD_RETRACT_FIBER = "0004"
CONTROL_WORD_INSERT_FIBER = "0008"
CONTROL_WORD_MOVE_TO_TARGET = "0010"
CONTROL_WORD_CLEAN_INSIDE = "0120"
CONTROL_WORD_CLEAN_OUTSIDE = "0220"
CONTROL_WORD_CLEAN_BOTH = "0320"

_CLEAN_MODES = {
    "inside": CONTROL_WORD_CLEAN_INSIDE,
    "outside": CONTROL_WORD_CLEAN_OUTSIDE,
    "both": CONTROL_WORD_CLEAN_BOTH,
}

# Valid position indexes
PORT_A_POSITIONS: Tuple[int, ...] = tuple(range(1, 14))  # 1-13
PORT_B_POSITIONS: Tuple[int, ...] = tuple(range(1, 6))  # 1-5

# Insertion/removal axis state, as reported in read_status()
INSERTION_STATE_REMOVED = 1
INSERTION_STATE_INSERTED = 3

# Alarms_0_15 bit 12 (0x1000): "M01 origin return timeout"
ALM_M01_ORIGIN_RETURN_TIMEOUT = 0x1000
# messages_0_15 bit 2 (0x0004): "Automatic operation is available"
MSG_AUTOMATIC_OPERATION_AVAILABLE = 0x0004


@dataclass
class SwitcherStatus:
    """Parsed reply from the "read device status" command.
    """

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


def _decode_32bit_swapped(hex8: str) -> int:
    """Decode an 8-hex-char PLC word pair into an integer.

    The PLC transmits 32-bit values as two 16-bit words in
    lower-word-then-upper-word order, e.g. ``"AFC80000"`` means
    ``0x0000AFC8`` (= 45000).
    """
    return int(hex8[4:8] + hex8[0:4], 16)


def _encode_32bit_swapped(value: int) -> str:
    """Inverse of `_decode_32bit_swapped`."""
    hex8 = f"{value:08X}"
    return hex8[4:8] + hex8[0:4]


def _position_to_code(position: int) -> str:
    """Convert a 1-based port position index to its 4-digit target code."""
    return f"{100 + position:04d}"


def _code_to_position(code: int) -> int:
    """Inverse of `_position_to_code`."""
    return code - 100


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
        self.sock: socket.socket | None = None
        self.host: str | None = None
        self.port: int | None = None
        self.read_timeout = read_timeout
        self._homed = False
        self._last_reply: str | None = None

    def connect(self, host: str, port: int = DEFAULT_PORT) -> None:  # pylint: disable=W0221
        """Open a TCP connection to the PLC.

        :param str host: PLC IP address (e.g. "192.168.0.183").
        :param int port: PLC port number (9001 for the first unit).
        """
        if self.is_connected():
            self.disconnect()

        self.report_info(f"Connecting to fiber switcher at {host}:{port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.read_timeout)
        try:
            sock.connect((host, port))
        except OSError as exc:
            sock.close()
            self.report_error(f"Could not connect to {host}:{port}: {exc}")
            raise ConnectionError(f"Could not connect to {host}:{port}") from exc

        self.sock = sock
        self.host = host
        self.port = port
        self._set_connected(True)
        self.report_info("Fiber switcher connected")

    def disconnect(self) -> None:
        """Close the TCP connection to the PLC."""
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        self._set_connected(False)
        self.report_info("Fiber switcher disconnected")

    def _require_connection(self) -> None:
        if not self.is_connected() or self.sock is None:
            raise RuntimeError("Fiber switcher is not connected")

    def _raw_send(self, command: str) -> None:
        self.sock.sendall(command.encode("ascii"))
        self.report_debug(f"Sent: {command}")

    def _recv_exact(self, count: int) -> str:
        chunks = []
        remaining = count
        while remaining > 0:
            chunk = self.sock.recv(remaining)
            if not chunk:
                raise ConnectionError("Fiber switcher closed the connection")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks).decode("ascii")

    def _raw_recv_frame(self) -> str:
        """Read one complete reply, using the embedded data-length field."""
        header = self._recv_exact(_HEADER_LEN)
        data_length = int(header[14:18], 16)
        rest = self._recv_exact(data_length)
        reply = header + rest
        self.report_debug(f"Received: {reply}")
        return reply

    def _send_command(self, command: str) -> bool:  # pylint: disable=W0221
        """Send a raw command string. See `_transact` for normal use."""
        self._require_connection()
        with self.lock:
            self._raw_send(command)
        return True

    def _read_reply(self) -> str:
        """Read one raw reply frame. See `_transact` for normal use."""
        self._require_connection()
        with self.lock:
            return self._raw_recv_frame()

    def _transact(self, command: str) -> str:
        """Send a command and return its reply, checking the PLC end code.

        This locks around the send and the receive together so concurrent
        callers cannot interleave requests on the shared TCP stream.
        """
        self._require_connection()
        with self.lock:
            self._raw_send(command)
            reply = self._raw_recv_frame()
        self._last_reply = reply
        end_code = reply[_HEADER_LEN : _HEADER_LEN + 4]
        if end_code != _ACK_OK:
            message = f"PLC returned error end code {end_code!r} for command {command!r}"
            self.report_error(message)
            raise RuntimeError(message)
        self._set_status((0, "Command acknowledged"))
        return reply

    def read_status(self) -> SwitcherStatus:
        """Read and parse the PLC's full device-status reply."""
        reply = self._transact(_READ_STATUS_CMD)
        payload = reply[_HEADER_LEN + 4 :]
        status = SwitcherStatus(
            port_a_target=int(payload[0:4]),
            port_b_target=int(payload[4:8]),
            in_position=payload[8:12] == "0001",
            insertion_state=int(payload[12:16], 16),
            horizontal_axis_position=_decode_32bit_swapped(payload[16:24]),
            insertion_axis_position=_decode_32bit_swapped(payload[24:32]),
            alarms_0_15=int(payload[40:44], 16),
            alarms_16_31=int(payload[44:48], 16),
            messages_0_15=int(payload[48:52], 16),
            messages_16_31=int(payload[52:56], 16),
        )
        return status

    def home(
        self,
        wait: bool = True,
        timeout: float = 60.0,
        poll_interval: float = 0.5,
    ) -> bool:
        """Start return-to-origin and, by default, wait for it to finish.

        The vendor document does not expose a dedicated "homed" status bit,
        so completion is inferred from the target/current position-match
        flag (``in_position``) together with the documented origin-return
        timeout alarm bit. Always call this once after PLC power-on before
        issuing any motion command.
        """
        self._transact(_CONTROL_WORD_PREFIX + CONTROL_WORD_HOME)
        self.report_info("Return-to-origin command sent")
        self._homed = False
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
        """Best-effort homed flag; see `home` for caveats."""
        return self._homed

    def is_loop_closed(self) -> bool:
        """Return the PLC's target/current position-match flag.

        The switcher's positioning loop is always closed;
        """
        return self.read_status().in_position

    def close_loop(self) -> bool:
        """No-op: this device has no selectable open/closed-loop mode."""
        self.report_warning("close_loop() is a no-op; this device has no open-loop mode")
        return self.is_loop_closed()

    def get_pos(self, axis: str = "A") -> int:  # pylint: disable=W0221
        """Return the current target position index (1-based) for ``axis``.

        :param str axis: "A" or "B".
        """
        axis = axis.upper()
        status = self.read_status()
        if axis == "A":
            return _code_to_position(status.port_a_target)
        if axis == "B":
            return _code_to_position(status.port_b_target)
        raise ValueError(f"Unknown axis {axis!r}; expected 'A' or 'B'")

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
        status = self.read_status()
        port_a = position if axis == "A" else _code_to_position(status.port_a_target)
        port_b = position if axis == "B" else _code_to_position(status.port_b_target)
        return self.set_target_positions(port_a, port_b)

    def get_limits(self) -> Dict[str, Tuple[int, int]]:
        """Return the documented position-index range for each port."""
        return {
            "A": (PORT_A_POSITIONS[0], PORT_A_POSITIONS[-1]),
            "B": (PORT_B_POSITIONS[0], PORT_B_POSITIONS[-1]),
        }

    def set_target_positions(self, port_a: int, port_b: int) -> bool:
        """Set both port target positions in a single command (section 2).

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
        command = _SET_TARGET_PREFIX + _position_to_code(port_a) + _position_to_code(port_b)
        self._transact(command)
        return True

    def stop(self) -> bool:
        """Stop motor motion in progress (section 6)."""
        self._transact(_CONTROL_WORD_PREFIX + CONTROL_WORD_STOP)
        return True

    def retract_fiber(self) -> bool:
        """Retract the fiber at the current position (section 4).

        Only works in automatic operation mode, per the vendor document.
        """
        self._transact(_CONTROL_WORD_PREFIX + CONTROL_WORD_RETRACT_FIBER)
        return True

    def insert_fiber(self) -> bool:
        """Insert the fiber at the current target position (section 5).

        Only works in automatic operation mode, per the vendor document.
        """
        self._transact(_CONTROL_WORD_PREFIX + CONTROL_WORD_INSERT_FIBER)
        return True

    def move_to_target(self) -> bool:
        """Move to the current target fiber position without inserting it
        (section 7)."""
        self._transact(_CONTROL_WORD_PREFIX + CONTROL_WORD_MOVE_TO_TARGET)
        return True

    def clean(self, mode: str = "both") -> bool:
        """Start a cleaning cycle (section 8).

        :param str mode: One of "inside", "outside", or "both".
        """
        if mode not in _CLEAN_MODES:
            raise ValueError(f"Unknown clean mode {mode!r}; expected one of {sorted(_CLEAN_MODES)}")
        self._transact(_CONTROL_WORD_PREFIX + _CLEAN_MODES[mode])
        return True

    def move_to_rearmost(self) -> bool:
        """Move the insertion/removal axis to the rearmost traversing
        position. Only valid once homing has completed."""
        self._transact(_REARMOST_CMD)
        return True

    def open_air_purge(self) -> bool:
        """Open the air purge valve."""
        self._transact(_AIR_PURGE_PREFIX + "1")
        return True

    def close_air_purge(self) -> bool:
        """Close the air purge valve."""
        self._transact(_AIR_PURGE_PREFIX + "0")
        return True

    def write_axis_register(self, register: str, value_mm: float) -> bool:
        """Write a 0.01 mm-resolution axis-setting register.

        :param str register: Only
            :data:`AXIS_REGISTER_RETRACT_DISTANCE`,
            :data:`AXIS_REGISTER_CAMERA_INSERTION_POSITION`, and
            :data:`AXIS_REGISTER_NONCAMERA_INSERTION_POSITION` are
            documented by the vendor.
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
        command = _AXIS_REGISTER_PREFIX.format(register=register) + _encode_32bit_swapped(raw)
        self._transact(command)
        return True

    def set_retract_distance_mm(self, value_mm: float) -> bool:
        """Set the insertion/removal axis's retract-back distance."""
        return self.write_axis_register(AXIS_REGISTER_RETRACT_DISTANCE, value_mm)

    def set_camera_insertion_position_mm(self, value_mm: float) -> bool:
        """Set the insertion position used for camera observation ports."""
        return self.write_axis_register(AXIS_REGISTER_CAMERA_INSERTION_POSITION, value_mm)

    def set_noncamera_insertion_position_mm(self, value_mm: float) -> bool:
        """Set the insertion position used for non-camera observation ports."""
        return self.write_axis_register(AXIS_REGISTER_NONCAMERA_INSERTION_POSITION, value_mm)

    def __enter__(self) -> "FiberSwitcher":
        return self

    def __exit__(self, *_: object) -> None:
        self.disconnect()
