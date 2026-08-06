"""Tests for the fiber switcher driver."""

from unittest.mock import MagicMock, patch

import pytest

import fiber_switcher as fs
from fiber_switcher import FiberSwitcher

ACK = "D00000FF03FF0000040000"
STATUS_REPLY = (
    "D00000FF03FF00003C00000104010200010001AFC8000003E80000753000000000000000040010"
)


def _mock_connected_switcher(recv_frames):
    """Return a FiberSwitcher wired to a mock socket that yields the given
    ASCII frames (each frame split into header(18)+rest bytes) in order."""
    switcher = FiberSwitcher(log=False)
    with patch("fiber_switcher.socket.socket") as mock_socket_cls:
        sock = mock_socket_cls.return_value
        recv_side_effect = []
        for frame in recv_frames:
            recv_side_effect.append(frame[:18].encode("ascii"))
            recv_side_effect.append(frame[18:].encode("ascii"))
        sock.recv.side_effect = recv_side_effect
        switcher.connect("127.0.0.1", 9001)
    return switcher, sock


def test_connect_and_disconnect():
    switcher, sock = _mock_connected_switcher([])
    assert switcher.is_connected()
    switcher.disconnect()
    sock.close.assert_called_once()
    assert not switcher.is_connected()


def test_read_status_parses_all_fields():
    switcher, sock = _mock_connected_switcher([STATUS_REPLY])
    status = switcher.read_status()

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


def test_set_target_positions_builds_expected_command():
    switcher, sock = _mock_connected_switcher([ACK])
    assert switcher.set_target_positions(port_a=5, port_b=2)
    sent = sock.sendall.call_args[0][0].decode("ascii")
    assert sent == "500000FF03FF000020000014010000W*000014000201050102"


@pytest.mark.parametrize("position", [0, 14, -1])
def test_set_target_positions_rejects_bad_port_a(position):
    switcher, _sock = _mock_connected_switcher([])
    with pytest.raises(ValueError, match="Port A"):
        switcher.set_target_positions(port_a=position, port_b=1)


@pytest.mark.parametrize("position", [0, 6, -1])
def test_set_target_positions_rejects_bad_port_b(position):
    switcher, _sock = _mock_connected_switcher([])
    with pytest.raises(ValueError, match="Port B"):
        switcher.set_target_positions(port_a=1, port_b=position)


def test_get_pos_and_set_pos_roundtrip():
    switcher, sock = _mock_connected_switcher([STATUS_REPLY, STATUS_REPLY, STATUS_REPLY, ACK])
    assert switcher.get_pos("A") == 4
    assert switcher.get_pos("B") == 2

    switcher.set_pos("A", 7)
    sent = sock.sendall.call_args[0][0].decode("ascii")
    # Port B (2, from the status reply's target 0102) is preserved.
    assert sent == "500000FF03FF000020000014010000W*000014000201070102"


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
    switcher, sock = _mock_connected_switcher([ACK])
    assert getattr(switcher, method)()
    sent = sock.sendall.call_args[0][0].decode("ascii")
    assert sent == "500000FF03FF00001C0000140200000100W*000012" + code


@pytest.mark.parametrize(
    "mode,code",
    [
        ("inside", fs.CONTROL_WORD_CLEAN_INSIDE),
        ("outside", fs.CONTROL_WORD_CLEAN_OUTSIDE),
        ("both", fs.CONTROL_WORD_CLEAN_BOTH),
    ],
)
def test_clean_modes(mode, code):
    switcher, sock = _mock_connected_switcher([ACK])
    assert switcher.clean(mode)
    sent = sock.sendall.call_args[0][0].decode("ascii")
    assert sent == "500000FF03FF00001C0000140200000100W*000012" + code


def test_clean_rejects_unknown_mode():
    switcher, _sock = _mock_connected_switcher([])
    with pytest.raises(ValueError, match="Unknown clean mode"):
        switcher.clean("sideways")


def test_move_to_rearmost():
    switcher, sock = _mock_connected_switcher([ACK])
    assert switcher.move_to_rearmost()
    sent = sock.sendall.call_args[0][0].decode("ascii")
    assert sent == "500000FF03FF00001C000014010000D*00020000010005"


def test_air_purge_open_and_close():
    switcher, sock = _mock_connected_switcher([ACK, ACK])
    switcher.open_air_purge()
    assert (
        sock.sendall.call_args_list[0][0][0].decode("ascii")
        == "500000FF03FF000019000014010001M*00038100011"
    )
    switcher.close_air_purge()
    assert (
        sock.sendall.call_args_list[1][0][0].decode("ascii")
        == "500000FF03FF000019000014010001M*00038100010"
    )


def test_write_axis_register_encodes_value():
    switcher, sock = _mock_connected_switcher([ACK])
    assert switcher.set_retract_distance_mm(2.56)
    sent = sock.sendall.call_args[0][0].decode("ascii")
    assert sent == "500000FF03FF000020000014010000D*001228000201000000"


def test_set_camera_and_noncamera_insertion_position():
    switcher, sock = _mock_connected_switcher([ACK, ACK])
    switcher.set_camera_insertion_position_mm(22.40)
    assert (
        sock.sendall.call_args_list[0][0][0].decode("ascii")
        == "500000FF03FF000020000014010000D*001250000208C00000"
    )
    switcher.set_noncamera_insertion_position_mm(34.30)
    assert (
        sock.sendall.call_args_list[1][0][0].decode("ascii")
        == "500000FF03FF000020000014010000D*00125800020D660000"
    )


def test_write_axis_register_rejects_bad_register():
    switcher, _sock = _mock_connected_switcher([])
    with pytest.raises(ValueError, match="4-digit"):
        switcher.write_axis_register("abcd", 1.0)


def test_error_end_code_raises():
    error_reply = "D00000FF03FF0000040001"  # end code 0001 = error
    switcher, _sock = _mock_connected_switcher([error_reply])
    with pytest.raises(RuntimeError, match="error end code"):
        switcher.stop()


def test_home_waits_for_in_position():
    switcher, _sock = _mock_connected_switcher([ACK, STATUS_REPLY])
    assert switcher.home(poll_interval=0)
    assert switcher.is_homed()


def test_home_raises_on_origin_timeout_alarm():
    alarm_status = (
        "D00000FF03FF00003C00000104010200010001AFC8000003E80000753000001000000000040010"
    )
    switcher, _sock = _mock_connected_switcher([ACK, alarm_status])
    with pytest.raises(RuntimeError, match="origin-return timeout"):
        switcher.home(poll_interval=0)


def test_is_loop_closed_reflects_in_position_flag():
    switcher, _sock = _mock_connected_switcher([STATUS_REPLY])
    assert switcher.is_loop_closed() is True


def test_close_loop_is_a_reporting_noop():
    switcher, _sock = _mock_connected_switcher([STATUS_REPLY])
    assert switcher.close_loop() is True


def test_get_limits():
    switcher, _sock = _mock_connected_switcher([])
    assert switcher.get_limits() == {"A": (1, 13), "B": (1, 5)}


def test_operations_require_connection():
    switcher = FiberSwitcher(log=False)
    with pytest.raises(RuntimeError, match="not connected"):
        switcher.read_status()
