#!/usr/bin/env python3
"""ROS 2 + serial integration test using only an internally created PTY.

Run after sourcing ROS Humble and this workspace's install/setup.bash:
    python3 tests/test_serial_ros.py

No physical device path is accepted. This launches two serial hand_nodes in an
isolated ROS domain, supplies test-only RS485 replies, and cleans up every child.
The test has a 60-second execution deadline plus bounded process cleanup.
"""

import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback


from angle_limits import ANGLE_MIN, ANGLE_MAX


NAMESPACE = '/inspire/ptytest_left_' + str(os.getpid())
RIGHT_NAMESPACE = '/inspire/ptytest_right_' + str(os.getpid())
INITIAL_ANGLES = [(lo + hi) // 2 for lo, hi in zip(ANGLE_MIN, ANGLE_MAX)]
FEEDBACK = {
    'angle_actual': ('angle_raw', 'angle_stamp'),
    'force_actual': ('force_raw', 'force_stamp'),
    'current_actual': ('current_ma', 'current_stamp'),
    'temperature': ('temperature_c', 'temperature_stamp'),
    'motor_status': ('motor_status', 'motor_status_stamp'),
    'error_code': ('error_code', 'error_stamp'),
}
# Independent wire expectations, in the documented register protocol.
NAMED_COMMANDS = {'angle_set': (1080, INITIAL_ANGLES), 'speed_set': (1108, [100] * 13),
                  'force_set': (1094, [90] * 13), 'current_limit_set': (1024, [80] * 13),
                  'mode_set': (1220, [0] * 13)}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class PseudoHand:
    """A strict RS485 peer; a lost ACK still applies the target to the device."""

    def __init__(self, angle_ack=1, stop_ack=1, single_angle_ack=1):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        require(self.port.startswith('/dev/pts/') and self.port[9:].isdigit(),
                'Refusing a path that is not an internally created PTY')
        os.set_blocking(self.master, False)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.requests = []
        self.error = None
        self.drop_next_angle_ack = False
        self.angle_ack = angle_ack
        self.stop_ack = stop_ack
        self.single_angle_ack = single_angle_ack
        self.ignore_stop_clear = False
        self.angle_lower_limits = {}
        self.registers = {}
        for base in (1024, 1080, 1094, 1108, 1136, 1150, 1164, 1178, 1192, 1206, 1220):
            for index in range(13):
                self.registers[base + index] = 25 if base == 1206 else 0
        for index, value in enumerate(INITIAL_ANGLES):
            self.registers[1080 + index] = value
            self.registers[1136 + index] = value
        for address in range(3000, 3034):
            self.registers[address] = 0
        for index in range(5):
            self.registers[3003 + 5 * index] = 0x3456
            self.registers[3004 + 5 * index] = 0x12
        self.registers[1003] = 0
        self.registers[1300] = 0
        self.registers[1301] = 0

    def start(self):
        self.thread.start()

    def close(self):
        self.stopping.set()
        if self.thread.is_alive():
            self.thread.join(timeout=0.3)
        os.close(self.slave)
        os.close(self.master)

    def snapshot(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError('PTY emulator failed') from self.error
            return list(self.requests)

    def writes(self):
        return [frame for frame in self.snapshot() if frame['command'] == 0x12]

    def lose_next_ack(self):
        with self.lock:
            self.drop_next_angle_ack = True

    def _reply(self, device_id, command, address, payload):
        frame = bytearray((0x90, 0xEB, device_id, len(payload) + 3, command,
                           address & 0xff, address >> 8))
        frame.extend(payload)
        frame.append(sum(frame[2:]) & 0xff)
        remaining = memoryview(frame)
        deadline = time.monotonic() + 0.5
        while remaining and not self.stopping.is_set():
            require(time.monotonic() < deadline, 'PTY reply write timed out')
            try:
                written = os.write(self.master, remaining)
                remaining = remaining[written:]
            except BlockingIOError:
                select.select([], [self.master], [], 0.01)

    def _handle(self, frame):
        require(frame[:2] == b'\xeb\x90', 'Driver sent a bad request header')
        require((sum(frame[2:-1]) & 0xff) == frame[-1], 'Driver sent bad checksum')
        device_id, command = frame[2], frame[4]
        address = int.from_bytes(frame[5:7], 'little')
        require(device_id == 1, 'Driver sent unexpected hand ID')
        require(command in (0x11, 0x12), 'Driver sent an unknown command')
        with self.lock:
            self.requests.append({'command': command, 'address': address,
                                  'frame': bytes(frame)})
            if command == 0x11:
                require(len(frame) == 9 and frame[3] == 4, 'Bad read request size')
                byte_count = frame[7]
                require(byte_count == 2, 'Default serial backend must read one register at a time')
                require(byte_count > 0 and byte_count % 2 == 0, 'Invalid read size')
                values = [self.registers[address + i] for i in range(byte_count // 2)]
                payload = struct.pack('<' + 'h' * len(values), *values)
                should_drop = False
            else:
                data = frame[7:-1]
                require(len(data) > 0 and len(data) % 2 == 0, 'Invalid write data size')
                values = struct.unpack('<' + 'h' * (len(data) // 2), data)
                is_angle = 1080 <= address <= 1092
                require(is_angle or address in (1003, 1024, 1094, 1108, 1220, 1301),
                        'Unexpected register write in integration test')
                if address not in (1003, 1301):
                    require((is_angle and len(values) == 1) or (len(values) == 13 and
                            (not is_angle or address == 1080)), 'Invalid joint write size/address')
                    if is_angle:
                        require(-1 not in values, 'Unverified -1 sentinel reached physical wire')
                    for index, value in enumerate(values):
                        self.registers[address + index] = value
                        # The real left hand accepts targets while stopped,
                        # but only a new angle command after stop=0 can move.
                        if is_angle and self.registers[1301] == 0:
                            channel = address - 1080 + index
                            self.registers[1136 + channel] = max(
                                value, self.angle_lower_limits.get(channel, -32768))
                else:
                    require(values in ((0,), (1,)) if address == 1301 else values == (1,),
                            'Stop flag must be 0/1; clear-errors must be 1')
                    if not (address == 1301 and values == (0,) and self.ignore_stop_clear):
                        self.registers[address] = values[0]
                    if address == 1003:
                        for index in range(13):
                            self.registers[1178 + index] = 0
                should_drop = is_angle and self.drop_next_angle_ack
                if should_drop:
                    self.drop_next_angle_ack = False
                payload = bytes([self.single_angle_ack if is_angle and len(values) == 1 else
                                 self.angle_ack if is_angle else
                                 self.stop_ack if address == 1301 else 1])
        if not should_drop:
            self._reply(device_id, command, address, payload)

    def _run(self):
        buffered = bytearray()
        try:
            while not self.stopping.is_set():
                readable, _, _ = select.select([self.master], [], [], 0.02)
                if not readable:
                    continue
                try:
                    chunk = os.read(self.master, 4096)
                except BlockingIOError:
                    continue
                except OSError as error:
                    if error.errno == errno.EIO:
                        self.stopping.wait(0.01)
                        continue
                    raise
                if not chunk:
                    continue
                buffered.extend(chunk)
                while len(buffered) >= 4:
                    require(buffered[:2] == b'\xeb\x90', 'Unexpected bytes written by driver')
                    frame_size = buffered[3] + 5
                    require(9 <= frame_size <= 34, 'Invalid request length')
                    if len(buffered) < frame_size:
                        break
                    frame = bytes(buffered[:frame_size])
                    del buffered[:frame_size]
                    self._handle(frame)
        except Exception as error:
            with self.lock:
                self.error = error


class RosHarness:
    def __init__(self, peer, child):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rh5dg2_interfaces.msg import CommandResult, JointCommand, JointFeedback
        from rh5dg2_interfaces.srv import GetState, SetEnabled, SetValues
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.peer = peer
        self.child = child
        self.node = rclpy.create_node('serial_pty_test_client', namespace=NAMESPACE)
        self.types = {'get_state': GetState, 'set_enabled': SetEnabled,
                      'set_values': SetValues, 'stop': Trigger, 'clear_stop': Trigger,
                      'clear_errors': Trigger, 'reconnect': Trigger}
        self.clients = {name: self.node.create_client(service_type, name)
                        for name, service_type in self.types.items()}
        self.feedback = {}
        self.results = {}
        self.command_id = 0
        self.command_type = JointCommand
        self.feedback_type = JointFeedback
        self.feedback_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [self.node.create_subscription(
            JointFeedback, topic, lambda msg, topic=topic: self.feedback.__setitem__(topic, msg),
            self.feedback_qos) for topic in FEEDBACK]
        self.subscriptions.append(self.node.create_subscription(
            CommandResult, 'command_result', lambda msg: self.results.__setitem__(msg.command_id, msg), 10))
        self.publishers = {topic: self.node.create_publisher(JointCommand, topic, 1)
                           for topic in NAMED_COMMANDS}

    def check(self):
        self.peer.snapshot()
        require(self.child.poll() is None, 'hand_node exited unexpectedly')

    def call(self, name, **fields):
        client = self.clients[name]
        deadline = time.monotonic() + 3.0
        while not client.wait_for_service(timeout_sec=0.05):
            self.check()
            require(time.monotonic() < deadline, 'Service discovery timed out: ' + name)
        request = self.types[name].Request()
        for field, value in fields.items():
            setattr(request, field, value)
        future = client.call_async(request)
        try:
            while not future.done():
                self.check()
                require(time.monotonic() < deadline, 'Service response timed out: ' + name)
                self.rclpy.spin_once(self.node, timeout_sec=0.02)
            return future.result()
        finally:
            if not future.done():
                future.cancel()

    def wait_fresh(self, angle=None):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            response = self.call('get_state')
            if response.success and not response.state.stale:
                state = response.state
                require(state.connected and state.angle_valid and state.error_valid,
                        'Fresh state did not contain valid connected feedback')
                if angle is None or state.angle_raw[0] == angle:
                    return state
            self.pause(0.02)
        raise AssertionError('Fresh state with expected angle was not received')

    def pause(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check()
            self.rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.02, deadline - time.monotonic())))

    def wait(self, predicate, description):
        deadline = time.monotonic() + 3.0
        while not predicate():
            require(time.monotonic() < deadline, description)
            self.pause(0.01)

    def command(self, topic, values):
        publisher = self.publishers[topic]
        self.wait(lambda: publisher.get_subscription_count() > 0 and
                  self.node.count_publishers(NAMESPACE + '/command_result') > 0,
                  'Named command topics did not connect')
        self.command_id += 1
        message = self.command_type()
        message.command_id = self.command_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.values = values
        publisher.publish(message)
        self.wait(lambda: message.command_id in self.results, 'Named command had no result')
        return self.results.pop(message.command_id)

    def close(self):
        self.node.destroy_node()


def exercise(harness, peer):
    state = harness.wait_fresh()
    require(not state.enabled, 'Serial driver enabled writes at startup')
    require(list(state.angle_raw) == INITIAL_ANGLES, 'Initial serial feedback differs from emulated device')
    harness.pause(0.3)
    require(len(peer.snapshot()) >= 6, 'Driver did not collect initial feedback groups')
    require(not peer.writes(), 'Read-only startup emitted a register write')

    target = [-1] * 13
    target[0] = INITIAL_ANGLES[0] - 10
    rejected = harness.call('set_values', kind=0, values=target)
    require(not rejected.success, 'Disabled driver accepted angle command')
    require(not peer.writes(), 'Disabled command reached serial device')
    print('PASS serial startup and disabled gate issue no writes', flush=True)

    # Software enable/disable must not inspect, set, or clear emergency stop.
    with peer.lock:
        peer.registers[1301] = 2
    request_count = len(peer.snapshot())
    enabled = harness.call('set_enabled', enabled=True)
    require(enabled.success, 'Explicit enable failed: ' + enabled.message)
    require(harness.call('set_enabled', enabled=False).success, 'Software disable failed')
    require(not peer.writes() and not any(
        r['address'] == 1301 for r in peer.snapshot()[request_count:]),
        'Software enable/disable accessed the emergency-stop register')
    require(harness.call('clear_stop').success, 'Explicit emergency-stop clear failed')
    require(not harness.call('get_state').state.enabled, 'Clear-stop enabled commands')
    require(harness.call('set_enabled', enabled=True).success, 'Software enable failed')
    require(harness.call('set_enabled', enabled=True).success and len(peer.writes()) == 1,
            'Repeated enable wrote a register')
    require(harness.call('set_values', kind=0, values=[-1] * 13).success,
            'All-skipped angle command failed')
    require(len(peer.writes()) == 1, 'All-skipped angle command wrote registers')
    print('PASS software enable/disable never access emergency stop; only explicit clear-stop writes zero', flush=True)
    # Reproduce the physical baseline: saved 1900/1750 exceed command limits.
    # Also keep a saved -1 and a saved value different from measured feedback.
    skipped = {1081: 1700, 1082: -1, 1086: 1900, 1087: 1900,
               1088: 1900, 1089: 1900, 1090: 1750}
    with peer.lock:
        peer.registers.update(skipped)
    outside = target.copy()
    outside[0] = -32768
    outside[12] = 32767
    limited = harness.call('set_values', kind=0, values=outside)
    expected = outside.copy()
    expected[0], expected[12] = ANGLE_MIN[0], ANGLE_MAX[12]
    require(limited.success and list(limited.applied_values) == expected,
            'Manual lower/upper limits were not applied')
    require(list(limited.clamped) == [True] + [False] * 11 + [True], 'Incorrect clamp mask')
    require('clamped' in limited.message, 'Saturation was not reported')
    writes = peer.writes()
    require([(w['address'], struct.unpack('<h', w['frame'][7:-1])[0]) for w in writes[1:]]
            == [(1080, ANGLE_MIN[0]), (1092, ANGLE_MAX[12])], 'Unclamped or skipped target reached serial wire')
    previous_count = len(writes)
    accepted = harness.call('set_values', kind=0, values=target)
    require(accepted.success, 'Acknowledged target was rejected: ' + accepted.message)
    state = harness.wait_fresh(angle=INITIAL_ANGLES[0] - 10)
    require(state.enabled, 'Acknowledged target unexpectedly disabled command gate')
    require(len(peer.writes()) == previous_count + 1 and peer.writes()[-1]['address'] == 1080,
            'Expected exactly one angle register write')
    wire_values = list(struct.unpack('<h', peer.writes()[-1]['frame'][7:-1]))
    require(wire_values == [INITIAL_ANGLES[0] - 10], 'Sparse command must contain only the requested channel')
    with peer.lock:
        require(all(peer.registers[a] == v for a, v in skipped.items()),
                'Sparse command modified an untouched saved target')
    print('PASS sparse angles apply manual limits, report applied targets, and never rewrite skipped channels', flush=True)
    print('PASS explicit enable, acknowledged angle write, fresh serial feedback', flush=True)

    for topic, (address, values) in NAMED_COMMANDS.items():
        previous_count = len(peer.writes())
        result = harness.command(topic, values)
        require(result.accepted, topic + ' rejected: ' + result.message)
        writes = peer.writes()
        require(len(writes) == previous_count + 1 and writes[-1]['address'] == address,
                topic + ' did not write exactly its documented register')
        require(list(struct.unpack('<13h', writes[-1]['frame'][7:-1])) == values,
                topic + ' corrupted command values')
    writes_before_fault = len(peer.writes())
    harness.wait(lambda: len(harness.feedback) == len(FEEDBACK) and
                 all(msg.valid for msg in harness.feedback.values()), 'Separate feedback not ready')
    print('PASS all five named command topics write exactly one correct register with original values', flush=True)

    peer.lose_next_ack()
    target[0] = INITIAL_ANGLES[0] - 20
    target[4] = INITIAL_ANGLES[4] - 10  # Must not be sent after the first channel loses its ACK.
    failed = harness.call('set_values', kind=0, values=target)
    require(not failed.success, 'Lost write ACK was reported as success')
    require('remaining channels not sent' in failed.message, 'Partial failure lost its operation context')
    with peer.lock:
        require(peer.registers[1084] == INITIAL_ANGLES[4], 'Remaining target sent after earlier failure')
    # Read raw service results: stale/disconnected state is intentional here.
    response = harness.call('get_state')
    require(not response.success and response.state.stale,
            'Disconnected cached state was reported fresh')
    require(not response.state.connected and not response.state.enabled,
            'Lost ACK failed to disconnect and disarm driver')
    require(not response.state.angle_valid and not response.state.error_valid,
            'Failure left old feedback marked valid')
    require(len(peer.writes()) == writes_before_fault + 1, 'Lost ACK caused an immediate retry')
    harness.wait(lambda: all(not msg.valid for msg in harness.feedback.values()),
                 'Fault did not invalidate every separate feedback topic')
    for topic, (field, stamp_field) in FEEDBACK.items():
        message = harness.feedback[topic]
        require(list(message.values) == list(getattr(response.state, field)),
                topic + ' replaced cached values after fault')
        require(message.header.stamp == getattr(response.state, stamp_field),
                topic + ' changed acquisition time after fault')
    # A newly attached terminal must receive the retained invalid sample even
    # while serial polling has stopped; no heartbeat/extra register read is used.
    late_samples = []
    late_subscription = harness.node.create_subscription(
        harness.feedback_type, 'angle_actual', late_samples.append, harness.feedback_qos)
    harness.wait(lambda: bool(late_samples), 'Late subscriber did not receive fault state')
    require(not late_samples[-1].valid and
            late_samples[-1].header.stamp == response.state.angle_stamp,
            'Late subscriber received fresh-looking stale data')
    harness.node.destroy_subscription(late_subscription)
    print('PASS separate feedback retains values/stamps but becomes invalid, including for late subscribers', flush=True)
    requests_after_fault = len(peer.snapshot())
    harness.pause(0.3)
    require(len(peer.snapshot()) == requests_after_fault,
            'Driver automatically reconnected or retried after lost ACK')
    retry_rejected = harness.call('set_values', kind=0, values=target)
    require(not retry_rejected.success, 'Disconnected command was accepted')
    require(len(peer.writes()) == writes_before_fault + 1, 'Rejected disconnected command reached serial device')
    print('PASS lost ACK disconnects, invalidates feedback, disarms, and never retries', flush=True)

    reconnected = harness.call('reconnect')
    require(reconnected.success, 'Explicit reconnect failed: ' + reconnected.message)
    # The device applied the command before dropping its ACK. The new angle is
    # legitimate feedback, while re-sending that old command would be a bug.
    state = harness.wait_fresh(angle=INITIAL_ANGLES[0] - 20)
    require(not state.enabled, 'Reconnect automatically re-enabled writes')
    harness.pause(0.2)
    require(len(peer.writes()) == writes_before_fault + 1, 'Reconnect replayed an old target')
    harness.wait(lambda: all(msg.valid for msg in harness.feedback.values()) and
                 harness.feedback['angle_actual'].values[0] == INITIAL_ANGLES[0] - 20,
                 'Separate feedback failed to recover after reconnect')
    target[0] = INITIAL_ANGLES[0] - 30
    require(not harness.call('set_values', kind=0, values=target).success,
            'Reconnect accepted command without a new explicit enable')
    require(len(peer.writes()) == writes_before_fault + 1, 'Disarmed reconnect wrote target')
    print('PASS explicit reconnect refreshes state without replay or re-enable', flush=True)

    require(harness.call('set_enabled', enabled=False).success, 'Explicit disable failed')
    require(len(peer.writes()) == writes_before_fault + 1, 'Already-disabled gate wrote a register')
    require(harness.call('stop').success, 'PTY firmware stop was not acknowledged')
    writes = peer.writes()
    require(len(writes) == writes_before_fault + 2 and writes[-1]['address'] == 1301,
            'Explicit stop did not produce exactly one firmware stop on the PTY')
    require(not harness.call('get_state').state.enabled, 'Explicit stop left gate enabled')
    require(harness.call('clear_stop').success, 'Explicit stop-clear failed')
    print('PASS emergency stop and clear-stop are explicit operations on the PTY', flush=True)


def check_control_readback(peer):
    script = Path(__file__).resolve().parents[1] / 'scripts/probe_serial.py'
    before = len(peer.snapshot())
    result = subprocess.run(
        ['/usr/bin/python3', str(script), '--port', peer.port,
         '--control-readback', '--timeout', '0.1'],
        capture_output=True, text=True, timeout=5)
    require(result.returncode == 0, 'Readback probe failed: ' + result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    expected_addresses = list(range(1080, 1093)) + list(range(1136, 1149)) + list(range(1178, 1191))
    require([row['address'] for row in rows] == expected_addresses,
            'Readback probe omitted or added registers')
    require(all(row['words'] == 1 and row['frame_valid'] for row in rows),
            'Readback probe did not validate individual responses')
    require([row['values'][0] for row in rows] == INITIAL_ANGLES * 2 + [0] * 13,
            'Readback values do not match device registers')
    requests = peer.snapshot()[before:]
    require(len(requests) == 39 and all(frame['command'] == 0x11 for frame in requests),
            'Readback probe sent a write or repeated/omitted a request')
    print('PASS control-readback reads 39 target/actual/error registers with zero writes', flush=True)


def run_demo(namespace, values):
    # Run the real entry point in a new process, with test targets instead of
    # the user's editable example values. No driver RPCs are exposed to it here.
    code = ("import json, sys; from rh5dg2_py import angle_demo; "
            "angle_demo.TARGET_ANGLES = json.loads(sys.argv[1]); "
            "sys.exit(angle_demo.main(['--namespace', sys.argv[2]]))")
    result = subprocess.run([sys.executable, '-c', code, json.dumps(values), namespace],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=8)
    require(result.returncode == 0 and '已发布到' in result.stdout,
            'Topic demo failed: ' + result.stdout)


def check_topic_only_demo(harness, namespace):
    from rh5dg2_interfaces.msg import JointCommand

    # Only a subscriber exists at this namespace: any service call would fail.
    received = []
    topic_only = namespace + '_topic_only'
    subscription = harness.node.create_subscription(
        JointCommand, topic_only + '/angle_set', received.append, 1)
    target = [-1] * 13
    target[4] = ANGLE_MAX[4] + 150
    try:
        run_demo(topic_only, target)
        harness.wait(lambda: len(received) > 0, 'Topic-only demo message missing')
        harness.pause(0.2)
        require(len(received) == 1 and list(received[0].values) == target,
                'Demo must publish exactly one unmodified target')
        require(received[0].command_id > 0 and received[0].header.stamp.sec > 0,
                'Demo topic command lacks timestamp/ID')
    finally:
        harness.node.destroy_subscription(subscription)
    print('PASS demo publishes once with no state, enable, stop or clear-stop services present', flush=True)

def check_topic_demo(harness, peer, namespace):
    from rh5dg2_py import HandClient

    target = [-1] * 13
    with HandClient(namespace, timeout_sec=3.0) as hand:
        require(hand.get_state().enabled, 'Default startup did not open topic command input')
        previous_count = len(peer.writes())
        with peer.lock:
            peer.angle_lower_limits[4] = ANGLE_MIN[4] + 14
        for offset, goal in enumerate((ANGLE_MIN[4] - 10, ANGLE_MAX[4] + 150), 1):
            target[4] = goal
            run_demo(namespace, target)
            harness.wait(lambda: len(peer.writes()) == previous_count + offset,
                         'Demo did not reach serial wire')
            state = hand.get_state()
            require(state.connected and state.enabled, 'Demo exit disabled or disconnected driver')
        expected = [(1084, ANGLE_MIN[4]), (1084, ANGLE_MAX[4])]
        writes = peer.writes()[previous_count:]
        require([(w['address'], struct.unpack('<h', w['frame'][7:-1])[0]) for w in writes] == expected,
                'Demo must write only clamped angles, without retries or emergency-stop writes')
        with peer.lock:
            peer.angle_lower_limits.clear()
            for index, value in enumerate(INITIAL_ANGLES):
                peer.registers[1080 + index] = peer.registers[1136 + index] = value
        harness.wait(lambda: list(hand.get_state().angle_raw) == INITIAL_ANGLES,
                     'Test fixture feedback did not settle')
        hand.set_enabled(False)  # Test setup for the subsequent disabled-gate checks.
    print('PASS default-enabled driver accepts consecutive topic demos, even when a target is unreachable', flush=True)


def check_motor_error(harness, peer):
    # Motor errors still reject commands, without writing emergency stop.
    previous_count = len(peer.writes())
    require(harness.call('set_enabled', enabled=True).success, 'Enable failed before error feedback')
    with peer.lock:
        peer.registers[1178] = 1
    harness.wait(lambda: not harness.call('get_state').state.enabled,
                 'Motor error did not disable software commands')
    require(len(peer.writes()) == previous_count, 'Motor-error feedback wrote a register')
    require(harness.call('clear_errors').success, 'Clear-errors failed')
    harness.wait_fresh()


def main():
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from check_serial_interfaces import run as check_interfaces
    from run_integration import reserve_domain
    from rh5dg2_py import HandClient

    executable = Path(get_package_prefix('rh5dg2_driver')) / 'lib/rh5dg2_driver/hand_node'
    require(executable.is_file(), 'Build and source this workspace before running this test')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    peer = PseudoHand(angle_ack=32, stop_ack=2, single_angle_ack=2)
    right_peer = PseudoHand()
    children = []
    harness = None
    initialized = False
    previous_handler = signal.getsignal(signal.SIGALRM)

    def timed_out(_signum, _frame):
        raise TimeoutError('Serial ROS integration test exceeded its 60-second deadline')

    signal.signal(signal.SIGALRM, timed_out)
    signal.setitimer(signal.ITIMER_REAL, 60.0)
    with reserve_domain() as domain, tempfile.TemporaryDirectory(prefix='rh5dg2-serial-ros-') as temporary:
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        log_paths = []
        try:
            peer.start()
            right_peer.start()
            check_control_readback(peer)
            environment = dict(os.environ, ROS_LOG_DIR=temporary)
            # No transport parameter here: default startup must use real serial
            # and fail clearly if its configured path does not exist.
            missing = str(Path(temporary) / 'missing-serial-device')
            result = subprocess.run([str(executable), '--ros-args', '-p', 'port:=' + missing],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, env=environment, timeout=5)
            require(result.returncode != 0 and 'Cannot connect to hand serial port' in result.stdout
                    and missing in result.stdout and 'No such file' in result.stdout,
                    'Missing hardware did not fail clearly: ' + result.stdout)
            print('PASS absent serial device exits nonzero with device path and OS error', flush=True)
            for parameter, message in [
                ('hand_id:=0', 'hand_id outside allowed range'),
                ('angle_rate:=0.0', 'angle_rate must be in'),
            ]:
                result = subprocess.run(
                    [str(executable), '--ros-args', '-p', 'port:=' + missing, '-p', parameter],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, env=environment, timeout=5)
                require(result.returncode != 0 and message in result.stdout
                        and 'Cannot connect' not in result.stdout,
                        'Invalid configuration did not fail before opening port: ' + result.stdout)
            print('PASS invalid configuration fails before serial open', flush=True)
            for side, device, namespace in [('left', peer, NAMESPACE),
                                            ('right', right_peer, RIGHT_NAMESPACE)]:
                command = [str(executable), '--ros-args', '-r', '__ns:=' + namespace,
                           '-p', 'side:=' + side, '-p', 'port:=' + device.port,
                           '-p', 'angle_write_ack:=' + str(device.angle_ack),
                           '-p', 'stop_write_ack:=' + str(device.stop_ack),
                           '-p', 'single_angle_write_ack:=' + str(device.single_angle_ack),
                           '-p', 'serial_timeout_ms:=120']
                if side == 'left':
                    command += ['-p', 'enable_write:=false']
                log_path = Path(temporary) / (side + '_hand_node.log')
                log_paths.append(log_path)
                with log_path.open('w') as log:
                    children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                     env=environment, start_new_session=True))
            rclpy.init(args=[])
            initialized = True
            harness = RosHarness(peer, children[0])
            harness.wait_fresh()
            result = subprocess.run([str(executable), '--ros-args', '-p', 'port:=' + peer.port],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, env=environment, timeout=5)
            require(result.returncode != 0 and 'Cannot connect to hand serial port' in result.stdout,
                    'Already owned serial device did not fail clearly: ' + result.stdout)
            require(not peer.writes(), 'Competing startup sent a hardware write')
            print('PASS an already owned port fails startup without register writes', flush=True)
            exercise(harness, peer)
            check_topic_demo(harness, right_peer, RIGHT_NAMESPACE)
            right_write_count = len(right_peer.writes())
            check_interfaces(NAMESPACE, RIGHT_NAMESPACE)
            check_motor_error(harness, peer)
            peer.snapshot()
            right_peer.snapshot()
            require(len(right_peer.writes()) == right_write_count, 'Left-hand tests wrote registers on the other hand')
            with HandClient(RIGHT_NAMESPACE, timeout_sec=3.0) as right:
                right.set_enabled(True)
            os.killpg(children[1].pid, signal.SIGINT)
            require(children[1].wait(timeout=3.0) == 0, 'Driver shutdown failed')
            require(len(right_peer.writes()) == right_write_count, 'Enabled driver wrote emergency stop during shutdown')
            print('PASS enabled driver shutdown writes no registers', flush=True)
            # Test zero and other old values; none may prevent verified recovery.
            for old_flag in (0, -1, -21846):
                with peer.lock:
                    peer.registers[1301] = old_flag
                previous_count = len(peer.writes())
                cleared = harness.call('clear_stop')
                require(cleared.success, 'Verified recovery failed: ' + cleared.message)
                writes = peer.writes()
                require(len(writes) == previous_count + 1 and writes[-1]['address'] == 1301
                        and writes[-1]['frame'][7:-1] == bytes([0, 0]),
                        'Recovery wrote an angle target or repeated stop-clear')
                require(harness.call('set_enabled', enabled=False).success, 'Recovery cleanup failed')
            # An ACK is insufficient: a nonzero post-write value must still fail.
            with peer.lock:
                peer.registers[1301] = 2
                peer.ignore_stop_clear = True
            previous_count = len(peer.writes())
            refused = harness.call('clear_stop')
            require(not refused.success and 'Stop register readback mismatch' in refused.message
                    and 'readback=[02 00]' in refused.message,
                    'Clear-stop accepted an uncleared firmware stop flag')
            state = harness.call('get_state').state
            require(not state.enabled and not state.connected,
                    'Failed stop-clear must leave the driver disarmed and disconnected')
            harness.pause(0.2)
            writes = peer.writes()
            require(len(writes) == previous_count + 1 and writes[-1]['address'] == 1301
                    and writes[-1]['frame'][7:-1] == bytes([0, 0]),
                    'Failed stop-clear retried or emitted an angle command')
            print('PASS explicit stop-clear readback mismatch disconnects and never retries', flush=True)
            # Destroying a DDS observer can hold the Python GIL; perform this
            # observer-only test after both emulator streams are no longer polled.
            check_topic_only_demo(harness, RIGHT_NAMESPACE)
            print('PASS all serial ROS integration checks; no physical device opened', flush=True)
            return 0
        except Exception:
            traceback.print_exc()
            for log_path in log_paths:
                if log_path.exists():
                    print('\n' + log_path.name + ':\n' + log_path.read_text(), flush=True)
            return 1
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous_handler)
            # No cleanup RPCs are sent to an external graph. This isolated child
            # holds only the generated PTY and is terminated before peer teardown.
            for child in children:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=0.5)
            if harness is not None:
                harness.close()
            if initialized:
                rclpy.shutdown()
            peer.close()
            right_peer.close()


if __name__ == '__main__':
    raise SystemExit(main())
