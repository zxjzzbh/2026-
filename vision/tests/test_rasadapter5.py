import struct
import pytest

from carvision.rasadapter5 import crc8, frame, Decoder, RasAdapter


def test_crc_matches_manufacturer_real_board_telemetry():
    # Packet captured while passively listening to this user's board.
    data = bytes.fromhex('0003047b12')
    assert crc8(data) == 0xa2
    assert Decoder().feed(bytes.fromhex('aa550003047b12a200')) == [(0, bytes.fromhex('047b12'))]


def test_decoder_handles_split_frames_noise_and_bad_crc():
    decoder = Decoder()
    packet = frame(4, bytes.fromhex('01057206'))
    assert decoder.feed(b'noise' + packet[:3]) == []
    assert decoder.feed(packet[3:]) == [(4, bytes.fromhex('01057206'))]
    bad = packet[:-1] + bytes((packet[-1] ^ 1,))
    assert decoder.feed(bad + packet) == [(4, bytes.fromhex('01057206'))]


def test_steering_command_matches_vendor_field_order_and_rejects_guessed_extremes():
    board = RasAdapter()
    sent = []
    board.send = lambda func, payload: sent.append((func, payload))
    board.set_position(3, 1650, .02)
    assert sent == [(4, struct.pack('<BHBBH', 1, 20, 1, 3, 1650))]
    for channel, pulse in ((0, 1650), (7, 1650), (1, 500), (1, 2500)):
        with pytest.raises(ValueError):
            board.set_position(channel, pulse)
    assert len(sent) == 1


def test_esc_only_commands_s4_without_touching_gimbals_or_motor_driver_ports():
    board = RasAdapter()
    sent = []
    board.send = lambda function, payload: sent.append((function, payload))
    for pulse in (1500, 1575, 1300):
        board.set_esc(4, pulse)
    assert all(function == 4 and payload[4] == 4 for function, payload in sent)
    for channel in (1, 2, 3, 5, 6):
        with pytest.raises(ValueError):
            board.set_esc(channel, 1500)
    assert len(sent) == 3


def test_close_drains_last_neutral_frame_before_restoring_baud(monkeypatch):
    termios = pytest.importorskip('termios')
    calls = []
    monkeypatch.setattr(termios, 'tcdrain', lambda fd: calls.append(('drain', fd)))
    monkeypatch.setattr(termios, 'tcsetattr', lambda fd, when, mode: calls.append(('restore', fd, when)))
    import carvision.rasadapter5 as module
    monkeypatch.setattr(module.os, 'close', lambda fd: calls.append(('close', fd)))
    board = RasAdapter()
    board.fd, board.original = 42, [0] * 7
    board.close()
    assert calls == [('drain', 42), ('restore', 42, termios.TCSADRAIN), ('close', 42)]


def test_nonblocking_receive_race_can_resume_with_a_valid_packet(monkeypatch):
    import carvision.rasadapter5 as module
    board = RasAdapter()
    board.fd = 42
    monkeypatch.setattr(module.select, 'select', lambda *args: ([42], [], []))
    values = iter([BlockingIOError(11, 'not ready'), frame(4, bytes.fromhex('0105dc05'))])
    def read(*args):
        value = next(values)
        if isinstance(value, Exception):
            raise value
        return value
    monkeypatch.setattr(module.os, 'read', read)
    assert board.receive() == []
    assert board.receive() == [(4, bytes.fromhex('0105dc05'))]


def test_gimbal_reference_cannot_touch_steering_esc_or_unobserved_endpoints():
    board = RasAdapter()
    sent = []
    board.send = lambda function, payload: sent.append((function, payload))
    board.set_gimbal_reference(1, 1500, .3)
    board.set_gimbal_reference(2, 1550, .1)
    assert sent == [(4, struct.pack('<BHBBH', 1, 300, 1, 1, 1500)),
                    (4, struct.pack('<BHBBH', 1, 100, 1, 2, 1550))]
    for channel in (0, 3, 4, 5, 6):
        with pytest.raises(ValueError):
            board.set_gimbal_reference(channel, 1500)
    for pulse in (0, 1499, 1551, 2500):
        with pytest.raises(ValueError):
            board.set_gimbal_reference(1, pulse)
    assert len(sent) == 2


def test_tilt_alignment_checks_command_continuity_and_limits_each_step():
    board = RasAdapter()
    sent = []
    board.send = lambda function, payload: sent.append((function, payload))
    board.read_position = lambda channel: 1500
    board.align_tilt_step(1500, 1490)
    assert sent == [(4, struct.pack('<BHBBH', 1, 100, 1, 2, 1490))]
    for previous, pulse in ((1500, 1450), (1500, 1510), (1000, 990), (1560, 1550), (True, 1000)):
        with pytest.raises(ValueError):
            board.align_tilt_step(previous, pulse)
    board.read_position = lambda channel: 1495
    with pytest.raises(RuntimeError):
        board.align_tilt_step(1500, 1490)
    assert len(sent) == 1


def test_pan_alignment_cannot_jump_or_ignore_a_changed_stored_command():
    board = RasAdapter()
    sent = []
    board.send = lambda function, payload: sent.append((function, payload))
    board.read_position = lambda channel: 1550
    board.align_pan_step(1550, 1560)
    board.align_pan_step(1550, 1540)
    assert all(payload[4] == 1 for _, payload in sent)
    for previous, pulse in ((1550, 1600), (1550, 1550), (1750, 1760), (1450, 1440), (True, 1550)):
        with pytest.raises(ValueError):
            board.align_pan_step(previous, pulse)
    board.read_position = lambda channel: 1555
    with pytest.raises(RuntimeError):
        board.align_pan_step(1550, 1560)
    assert len(sent) == 2
