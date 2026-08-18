"""Tests for the fiber switcher driver.

`FiberSwitcher` delegates protocol I/O to `pymcprotocol.Type3E`, so these
tests mock that client rather than a raw socket and assert on the
device/value arguments passed to it.
"""

from unittest.mock import call, patch

import pytest
from pymcprotocol.mcprotocolerror import MCProtocolError

import fiber_switcher as fs
from fiber_switcher import FiberSwitcher


def _signed16(hex_str: str) -> int:
    """Convert a 4-hex-char word to the signed 16-bit int pymcprotocol
    would return for it."""
    value = int(hex_str, 16)
    return value - 0x10000 if value >= 0x8000 else value


# The 14 status words from the vendor document's worked read-status example.
STATUS_WORDS = [
    _signed16(h)
    for h in (
        "0104", "0102", "0001", "0001",
        "AFC8", "0000", "03E8", "0000",
        "7530", "0000",
        "0000", "0000", "0004", "0010",
    )
]


def _connected_switcher():
    """Return a FiberSwitcher whose `_client` is a mock, already "connected"."""
    with patch("fiber_switcher.Type3E"):
        switcher = FiberSwitcher(log=False)
    client = switcher._client  # pylint: disable=protected-access
    switcher.connect("127.0.0.1", 9001)
    return switcher, client


def test_connect_and_disconnect():
    switcher, client = _connected_switcher()
    assert switcher.is_connected()
    switcher.disconnect()
    client.close.assert_called_once()
    assert not switcher.is_connected()


def test_read_status_parses_all_fields():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS

    status = switcher.read_status()

    client.batchread_wordunits.assert_called_once_with("W0", 14)
    assert status.port_a_target == 104
    assert status.port_b_target == 102
    assert status.in_position is True
    assert status.insertion_state == 1
    assert status.horizontal_axis_position == 45000
    assert status.insertion_axis_position == 1000
    assert status.alarms_0_15 == 0
    assert status.alarms_16_31 == 0
    assert status.messages_0_15 == 0x0004
    assert status.messages_16_31 == 0x0010


def test_set_target_positions_builds_expected_values():
    switcher, client = _connected_switcher()
    assert switcher.set_target_positions(port_a=5, port_b=2)
    client.batchwrite_wordunits.assert_called_once_with("W0E", [0x0105, 0x0102])


@pytest.mark.parametrize("position", [0, 14, -1])
def test_set_target_positions_rejects_bad_port_a(position):
    switcher = FiberSwitcher(log=False)
    with pytest.raises(ValueError, match="Port A"):
        switcher.set_target_positions(port_a=position, port_b=1)


@pytest.mark.parametrize("position", [0, 6, -1])
def test_set_target_positions_rejects_bad_port_b(position):
    switcher = FiberSwitcher(log=False)
    with pytest.raises(ValueError, match="Port B"):
        switcher.set_target_positions(port_a=1, port_b=position)


def test_get_pos_and_set_pos_roundtrip():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS

    assert switcher.get_pos("A") == 4
    assert switcher.get_pos("B") == 2

    switcher.set_pos("A", 7)
    # Port B (2, from the mocked status's target 0x0102) is preserved.
    client.batchwrite_wordunits.assert_called_with("W0E", [0x0107, 0x0102])


@pytest.mark.parametrize(
    "method,code",
    [
        ("stop", fs.CONTROL_WORD_STOP),
        ("retract_fiber", fs.CONTROL_WORD_RETRACT_FIBER),
        ("insert_fiber", fs.CONTROL_WORD_INSERT_FIBER),
        ("move_to_target", fs.CONTROL_WORD_MOVE_TO_TARGET),
    ],
)
def test_control_word_commands(method, code):
    switcher, client = _connected_switcher()
    assert getattr(switcher, method)()
    client.randomwrite.assert_called_once_with(["W0C"], [code], [], [])


@pytest.mark.parametrize(
    "mode,code",
    [
        ("inside", fs.CONTROL_WORD_CLEAN_INSIDE),
        ("outside", fs.CONTROL_WORD_CLEAN_OUTSIDE),
        ("both", fs.CONTROL_WORD_CLEAN_BOTH),
    ],
)
def test_clean_modes(mode, code):
    switcher, client = _connected_switcher()
    assert switcher.clean(mode)
    client.randomwrite.assert_called_once_with(["W0C"], [code], [], [])


def test_clean_defaults_to_both():
    switcher, client = _connected_switcher()
    assert switcher.clean()
    client.randomwrite.assert_called_once_with(["W0C"], [fs.CONTROL_WORD_CLEAN_BOTH], [], [])


def test_clean_rejects_unknown_mode():
    switcher = FiberSwitcher(log=False)
    with pytest.raises(ValueError, match="Unknown clean mode"):
        switcher.clean("sideways")


def test_get_pos_reflects_clean_position_override():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS  # target still 104/102

    switcher.clean()

    # get_pos() must reflect the known post-clean position, not the PLC's
    # stale target registers.
    assert switcher.get_pos("A") == fs.CLEAN_POSITION_PORT_A
    assert switcher.get_pos("B") == fs.CLEAN_POSITION_PORT_B
    client.batchread_wordunits.assert_not_called()


def test_read_status_reflects_clean_position_override():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS  # raw registers still 104/102

    switcher.clean()
    status = switcher.read_status()

    # read_status() must agree with get_pos(), not echo the PLC's stale
    # target registers.
    assert status.port_a_target == 100 + fs.CLEAN_POSITION_PORT_A
    assert status.port_b_target == 100 + fs.CLEAN_POSITION_PORT_B


def test_set_target_positions_clears_clean_override():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS

    switcher.clean()
    switcher.set_target_positions(port_a=3, port_b=1)
    switcher.get_pos("A")

    # get_pos() must go back to reading the PLC live instead of returning
    # the cached clean-position override.
    client.batchread_wordunits.assert_called_once_with("W0", 14)


def test_home_clears_clean_override():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS

    switcher.clean()
    switcher.home(poll_interval=0)

    assert switcher.get_pos("A") == 4  # back to reading the PLC's own status
    client.batchread_wordunits.assert_called_with("W0", 14)


def test_move_to_rearmost():
    switcher, client = _connected_switcher()
    assert switcher.move_to_rearmost()
    client.batchwrite_wordunits.assert_called_once_with("D200", [5])


def test_air_purge_open_and_close():
    switcher, client = _connected_switcher()
    switcher.open_air_purge()
    switcher.close_air_purge()
    assert client.batchwrite_bitunits.call_args_list == [
        call("M381", [1]),
        call("M381", [0]),
    ]


def test_write_axis_register_encodes_value():
    switcher, client = _connected_switcher()
    assert switcher.set_retract_distance_mm(2.56)
    client.batchwrite_wordunits.assert_called_once_with("D1228", [0x0100, 0x0000])


def test_set_camera_and_noncamera_insertion_position():
    switcher, client = _connected_switcher()
    switcher.set_camera_insertion_position_mm(22.40)
    switcher.set_noncamera_insertion_position_mm(34.30)
    assert client.batchwrite_wordunits.call_args_list == [
        call("D1250", [0x08C0, 0x0000]),
        call("D1258", [0x0D66, 0x0000]),
    ]


def test_write_axis_register_rejects_bad_register():
    switcher = FiberSwitcher(log=False)
    with pytest.raises(ValueError, match="4-digit"):
        switcher.write_axis_register("abcd", 1.0)


def test_read_axis_register_decodes_value():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = [0x0100, 0x0000]
    assert switcher.get_retract_distance_mm() == 2.56
    client.batchread_wordunits.assert_called_once_with("D1228", 2)


def test_get_camera_and_noncamera_insertion_position():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = [0x08C0, 0x0000]
    assert switcher.get_camera_insertion_position_mm() == 22.40
    client.batchread_wordunits.return_value = [0x0D66, 0x0000]
    assert switcher.get_noncamera_insertion_position_mm() == 34.30
    assert client.batchread_wordunits.call_args_list == [
        call("D1250", 2),
        call("D1258", 2),
    ]


def test_read_axis_register_rejects_bad_register():
    switcher = FiberSwitcher(log=False)
    with pytest.raises(ValueError, match="4-digit"):
        switcher.read_axis_register("abcd")


def test_plc_error_propagates():
    switcher, client = _connected_switcher()
    client.randomwrite.side_effect = MCProtocolError(0x4031)
    with pytest.raises(MCProtocolError):
        switcher.stop()


def test_home_waits_for_in_position():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS  # in_position is True
    assert switcher.home(poll_interval=0)
    assert switcher.is_homed()
    client.randomwrite.assert_called_once_with(["W0C"], [fs.CONTROL_WORD_HOME], [], [])


def test_home_raises_on_origin_timeout_alarm():
    switcher, client = _connected_switcher()
    alarm_words = list(STATUS_WORDS)
    alarm_words[10] = _signed16("1000")  # alarms_0_15 bit 12
    client.batchread_wordunits.return_value = alarm_words
    with pytest.raises(RuntimeError, match="origin-return timeout"):
        switcher.home(poll_interval=0)


def test_is_loop_closed_reflects_in_position_flag():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS
    assert switcher.is_loop_closed() is True


def test_close_loop_is_a_reporting_noop():
    switcher, client = _connected_switcher()
    client.batchread_wordunits.return_value = STATUS_WORDS
    assert switcher.close_loop() is True


def test_get_limits():
    switcher = FiberSwitcher(log=False)
    assert switcher.get_limits() == {"A": (1, 13), "B": (1, 5)}


def test_operations_require_connection():
    switcher = FiberSwitcher(log=False)
    with pytest.raises(RuntimeError, match="not connected"):
        switcher.read_status()
