import math
import socket
import struct

import pytest

from main.udp_motor_bridge import UdpMotorSender, pack_motor_command


def receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(1.0)
    return sock


def test_udp_sender_uses_exact_network_float_payload():
    sock = receiver()
    sender = None
    try:
        sender = UdpMotorSender(*sock.getsockname())
        sender.send([1.5, -2.25])
        packet, _ = sock.recvfrom(64)
    finally:
        if sender is not None:
            sender.socket.close()
        sock.close()
    assert packet == struct.pack("!ff", 1.5, -2.25)
    assert len(packet) == 8


@pytest.mark.parametrize("values", [[], [1.0], [math.nan, 0.0], [0.0, math.inf]])
def test_invalid_motor_commands_are_rejected(values):
    with pytest.raises(ValueError):
        pack_motor_command(values)


def test_shutdown_transmits_zero_commands():
    sock = receiver()
    sender = UdpMotorSender(*sock.getsockname())
    sender.close_with_zero()
    packets = [sock.recvfrom(64)[0] for _ in range(3)]
    sock.close()
    assert packets == [struct.pack("!ff", 0.0, 0.0)] * 3
