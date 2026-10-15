#!/usr/bin/env python3
"""Bounded, read-only RH5DG2 serial probe, independent of ROS and the C++ driver.

Stop the driver first: this utility requires exclusive ownership of the port.
Only the protocol's READ (0x11) command is implemented. No motion, enable,
clear-error, stop, configuration or EEPROM writes are sent.
"""

import argparse
import fcntl
import json
import os
import select
import struct
import sys
import termios
import time


READS = (
    ('hand_id', 1000, 1), ('baud_setting', 1001, 1),
    ('angle_first', 1136, 1), ('angles', 1136, 13),
    ('forces', 1150, 13), ('currents', 1164, 13),
    ('errors', 1178, 13), ('motor_status', 1192, 13),
    ('temperatures', 1206, 13),
)

# Fixed read-only set for investigating a write with an uncertain outcome.
# Always read individual registers, including every channel rather than only
# the intended target: a block write may have affected other channels.
CONTROL_READBACK = (
    ('angle_targets', 1080, 13), ('angles', 1136, 13), ('errors', 1178, 13),
)


def read_request(hand_id, address, words):
    frame = bytearray((0xeb, 0x90, hand_id, 4, 0x11,
                       address & 255, address >> 8, words * 2))
    frame.append(sum(frame[2:]) & 255)
    return bytes(frame)


def transact(fd, hand_id, address, words, timeout):
    request = read_request(hand_id, address, words)
    termios.tcflush(fd, termios.TCIFLUSH)
    deadline = time.monotonic() + timeout
    sent = 0
    while sent < len(request):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError('Timeout sending read request')
        try:
            sent += os.write(fd, request[sent:])
        except BlockingIOError:
            pass
    received = bytearray()
    frame_size = 8 + 2 * words
    while time.monotonic() < deadline:
        if not select.select([fd], [], [], max(0, deadline - time.monotonic()))[0]:
            break
        try:
            chunk = os.read(fd, 512)
        except BlockingIOError:
            continue
        if not chunk:
            break
        received.extend(chunk)
        for offset in range(len(received) - frame_size + 1):
            frame = received[offset:offset + frame_size]
            if (frame[:7] != bytes((0x90, 0xeb, hand_id, 2 * words + 3,
                                    0x11, address & 255, address >> 8))
                    or sum(frame[2:-1]) & 255 != frame[-1]):
                continue
            return {'tx_hex': request.hex(' '), 'rx_hex': bytes(received).hex(' '),
                    'frame_hex': bytes(frame).hex(' '), 'frame_valid': True,
                    'values': list(struct.unpack('<' + 'h' * words, frame[7:-1]))}
        if len(received) > 4096:
            raise RuntimeError('Excessive serial input; no matching frame')
    return {'tx_hex': request.hex(' '), 'rx_hex': bytes(received).hex(' '),
            'frame_valid': False, 'error': 'No matching, checksummed response before deadline'}


def run(port, baud, hand_id, timeout, single_registers=False, control_readback=False):
    speed = getattr(termios, 'B' + str(baud), None)
    if speed is None:
        raise ValueError('Unsupported baud rate')
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[4] = settings[5] = speed
        settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIOFLUSH)
        # Bound recovery and discard delayed responses from the previous owner.
        hard_end = time.monotonic() + 3 * timeout
        quiet_end = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if now >= quiet_end:
                break
            if now >= hard_end:
                raise RuntimeError('Input did not become quiet; check other bus masters')
            if select.select([fd], [], [], min(quiet_end, hard_end) - now)[0]:
                os.read(fd, 512)
                quiet_end = time.monotonic() + timeout
        reads = CONTROL_READBACK if control_readback else READS
        if single_registers or control_readback:
            reads = tuple((f'{name}[{index}]', address + index, 1)
                          for name, address, words in reads
                          if name != 'angle_first' for index in range(words))
        for name, address, words in reads:
            result = transact(fd, hand_id, address, words, timeout)
            result.update(name=name, address=address, words=words)
            print(json.dumps(result), flush=True)
            # Do not allow a delayed reply to satisfy a subsequent request.
            if not result['frame_valid']:
                return 1
        return 0
    finally:
        if exclusive:
            fcntl.ioctl(fd, termios.TIOCNXCL)
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=115200,
                        choices=(9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600))
    parser.add_argument('--hand-id', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=0.25)
    parser.add_argument('--single-registers', action='store_true',
                        help='read each word separately to diagnose multi-register compatibility')
    parser.add_argument('--control-readback', action='store_true',
                        help='read all angle targets, actual angles and errors, one register at a time; no writes')
    args = parser.parse_args()
    if not 1 <= args.hand_id <= 254 or not 0.01 <= args.timeout <= 2:
        parser.error('hand-id must be 1..254 and timeout 0.01..2 seconds')
    try:
        return run(args.port, args.baud, args.hand_id, args.timeout,
                   args.single_registers, args.control_readback)
    except (OSError, ValueError, RuntimeError, termios.error) as error:
        print('Read-only probe failed: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
