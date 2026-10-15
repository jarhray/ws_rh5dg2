"""ROS interface checks called only by the suite owning both internal PTYs."""

from contextlib import ExitStack
from types import SimpleNamespace
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from rh5dg2_interfaces.msg import (
    CommandResult, HandCommand, HandState, JointCommand, JointFeedback, TouchState)
from rh5dg2_py import HandClient, HandError, HandRejectedError
from angle_limits import ANGLE_MIN, ANGLE_MAX


FEEDBACK = {
    'angle_actual': ('angle_raw', 'angle_stamp', '0.1 deg'),
    'force_actual': ('force_raw', 'force_stamp', 'g'),
    'current_actual': ('current_ma', 'current_stamp', 'mA'),
    'temperature': ('temperature_c', 'temperature_stamp', 'degC'),
    'motor_status': ('motor_status', 'motor_status_stamp', 'code'),
    'error_code': ('error_code', 'error_stamp', 'code'),
}
COMMAND_TOPICS = ('angle_set', 'speed_set', 'force_set', 'current_limit_set', 'mode_set')


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def report(message):
    print('PASS: ' + message, flush=True)


def wait_state(hand, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    last = 'no response'
    while time.monotonic() < deadline:
        try:
            state = hand.get_state()
            if predicate(state):
                return state
            last = state.status
        except HandError as exc:
            last = str(exc)
        time.sleep(0.04)
    raise AssertionError('{}: {} (deadline {} s)'.format(description, last, timeout))


def fresh(state):
    return (state.connected and not state.stale
            and state.angle_valid and state.error_valid)


def expect_rejected(operation, text):
    try:
        operation()
    except HandRejectedError as exc:
        require(text.lower() in str(exc).lower(),
                'expected rejection containing {!r}, got {}'.format(text, exc))
        return
    raise AssertionError('operation unexpectedly succeeded; expected ' + text)


class Monitor:
    """Own a separate context to check actual ROS messages, not client internals."""

    def __init__(self, left_namespace, right_namespace):
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node('rh5dg2_serial_acceptance', context=self.context,
                                     use_global_arguments=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.results = {}
        self.states = {}
        self.touches = {}
        self.feedback = {}
        self.left_namespace = left_namespace
        self.command_id = time.time_ns() % (2 ** 63)
        self.subscriptions = []
        feedback_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for side, namespace in [('left', left_namespace), ('right', right_namespace)]:
            for topic in FEEDBACK:
                self.subscriptions.append(self.node.create_subscription(
                    JointFeedback, namespace + '/' + topic,
                    lambda message, key=(side, topic): self.feedback.__setitem__(key, message),
                    feedback_qos))
            self.subscriptions.append(self.node.create_subscription(
                HandState, namespace + '/state',
                lambda message, side=side: self.states.__setitem__(side, message), 10))
            self.subscriptions.append(self.node.create_subscription(
                TouchState, namespace + '/touch',
                lambda message, side=side: self.touches.__setitem__(side, message),
                qos_profile_sensor_data))
        self.subscriptions.append(self.node.create_subscription(
            CommandResult, left_namespace + '/command_result',
            lambda message: self.results.__setitem__(message.command_id, message), 10))
        self.publisher = self.node.create_publisher(HandCommand, left_namespace + '/command', 10)
        self.named_publishers = {
            topic: self.node.create_publisher(JointCommand, left_namespace + '/' + topic, 1)
            for topic in COMMAND_TOPICS}

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        raise AssertionError('{} (deadline {} s)'.format(description, timeout))

    def command(self, values, timeout, offset_sec=0, zero_stamp=False, stamp_ns_override=None,
                topic=None):
        self.command_id += 1
        message = JointCommand() if topic else HandCommand()
        message.command_id = self.command_id
        if topic is None:
            message.kind = HandCommand.ANGLE
        # Generated fixed-size ROS arrays may iterate as numpy.int16 values;
        # generated message setters require built-in Python integers.
        message.values = [int(value) for value in values]
        if not zero_stamp:
            nanoseconds = (stamp_ns_override if stamp_ns_override is not None else
                           self.node.get_clock().now().nanoseconds + int(offset_sec * 1e9))
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
        publisher = self.named_publishers[topic] if topic else self.publisher
        publisher.publish(message)
        self.wait(lambda: message.command_id in self.results, timeout,
                  'no CommandResult for command_id={}'.format(message.command_id))
        result = self.results.pop(message.command_id)
        require(stamp_ns(result.header.stamp) > 0, 'CommandResult lacks timestamp')
        return result

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=0.0)
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def run(left_namespace, right_namespace):
    args = SimpleNamespace(left_namespace=left_namespace, right_namespace=right_namespace,
                           rpc_timeout=2.0, discovery_timeout=10.0, operation_timeout=4.0)
    require(all(name.startswith('/inspire/ptytest_') for name in (left_namespace, right_namespace)),
            'Only internally owned PTY test namespaces are accepted')
    require(args.left_namespace != args.right_namespace, 'left and right namespaces must differ')
    with ExitStack() as stack:
        left = stack.enter_context(HandClient(args.left_namespace, args.rpc_timeout))
        right = stack.enter_context(HandClient(args.right_namespace, args.rpc_timeout))
        left_start = wait_state(left, fresh, args.discovery_timeout, 'left serial did not become ready')
        right_start = wait_state(right, fresh, args.discovery_timeout, 'right serial did not become ready')
        verified_clients = [left, right]
        monitor = stack.enter_context(Monitor(left.namespace, right.namespace))
        try:
            require(not left_start.enabled and not right_start.enabled,
                    'launch fresh serial drivers with enable_write=false before running this script')
            require(all(name.startswith('left_') for name in left_start.joint_names),
                    'left joint_names lack left prefix')
            require(all(name.startswith('right_') for name in right_start.joint_names),
                    'right joint_names lack right prefix')
            original_left = list(left_start.angle_raw)
            original_right = list(right_start.angle_raw)
            expect_rejected(lambda: left.set_angle(original_left), 'disabled')
            expect_rejected(lambda: right.set_angle(original_right), 'disabled')
            report('both serial hands are fresh, named by side, and reject writes while disabled')

            all_valid = lambda s: fresh(s) and all(getattr(s, name + '_valid') for name in (
                'force', 'current', 'temperature', 'motor_status'))
            state = wait_state(left, all_valid, args.discovery_timeout, 'feedback blocks incomplete')
            for name in ['angle', 'force', 'current', 'temperature', 'motor_status', 'error']:
                acquired = stamp_ns(getattr(state, name + '_stamp'))
                require(0 < acquired <= stamp_ns(state.header.stamp),
                        '{} acquisition timestamp invalid'.format(name))
            require(list(state.force_sensor_present) == [True] * 6 + [False] * 4 + [True] * 3,
                    'force sensor presence mask differs from the documented channels')
            newer = wait_state(left, lambda s: fresh(s) and stamp_ns(s.angle_stamp) > stamp_ns(state.angle_stamp),
                               args.operation_timeout, 'angle acquisition timestamp did not advance')
            require(newer.sequence >= state.sequence, 'state sequence went backwards')
            report('service feedback has independent valid flags, advancing acquisition stamps, and sensor mask')

            monitor.wait(lambda: (
                set(monitor.states) == {'left', 'right'}
                and set(monitor.touches) == {'left', 'right'}
                and len(monitor.feedback) == 2 * len(FEEDBACK)
                and all(message.valid for message in monitor.feedback.values())
                and monitor.publisher.get_subscription_count() > 0
                and all(pub.get_subscription_count() > 0 for pub in monitor.named_publishers.values())
                and monitor.node.count_publishers(left.namespace + '/command_result') > 0),
                args.discovery_timeout, 'state/touch/command topics did not connect')
            for side in ['left', 'right']:
                require(fresh(monitor.states[side]), side + ' state topic is not fresh serial feedback')
                touch = monitor.touches[side]
                require(touch.valid, side + ' touch topic invalid')
                require(stamp_ns(touch.header.stamp) > 0, side + ' touch timestamp missing')
                require(list(touch.proximity) == [0x123456] * 5,
                        side + ' 24-bit proximity value was truncated or decoded incorrectly')
            report('both real ROS state/touch topics work and preserve 24-bit proximity 0x123456')

            for side, hand in [('left', left), ('right', right)]:
                cached = hand.get_state()
                for topic, (field, stamp_field, unit) in FEEDBACK.items():
                    message = monitor.feedback[side, topic]
                    require(message.valid, side + '/' + topic + ' invalid')
                    require(message.unit == unit, topic + ' has wrong unit')
                    require(list(message.values) == list(getattr(cached, field)),
                            topic + ' differs from aggregate feedback')
                    require(0 < stamp_ns(message.header.stamp) <= stamp_ns(getattr(cached, stamp_field)),
                            topic + ' does not use an acquisition timestamp')
                    require(list(message.joint_names) == list(cached.joint_names),
                            topic + ' channel order/side differs from aggregate feedback')
                    expected = list(cached.force_sensor_present) if topic == 'force_actual' else [True] * 13
                    require(list(message.channel_present) == expected, topic + ' wrong presence mask')
            report('six separate feedback topics on each hand match cached values, units, stamps and channel masks')

            named_values = dict(zip(COMMAND_TOPICS, (original_left, [100] * 13, [100] * 13,
                                                     [100] * 13, [0] * 13)))
            for topic, values in named_values.items():
                result = monitor.command(values, args.operation_timeout, topic=topic)
                require(not result.accepted and 'disabled' in result.message.lower(),
                        topic + ' bypassed disabled gate')
            report('all five named command topics enforce the disabled gate')

            left.set_enabled(True)
            require(left.get_state().enabled, 'left enable was not reflected in feedback')
            require(not right.get_state().enabled, 'left enable affected right hand')
            target = original_left.copy()
            target[4] += -10 if target[4] >= ANGLE_MAX[4] - 10 else 10
            left.set_angle(target)
            moved = wait_state(left, lambda s: fresh(s) and list(s.angle_raw) == target,
                               args.operation_timeout, 'left angle command did not reach feedback')
            require(list(right.get_state().angle_raw) == original_right,
                    'left command changed right hand')
            require(moved.enabled, 'left disarmed during a valid command')
            report('enabled left hand moved 1 degree; right hand stayed disabled and unchanged')

            invalid = target.copy()
            invalid[4] = 30000
            result = left.set_angle(invalid)
            expected = target.copy()
            expected[4] = ANGLE_MAX[4]
            require(list(result.applied_values) == expected and result.clamped[4],
                    'Service did not report clamped target')
            wait_state(left, lambda s: list(s.angle_raw) == expected,
                       args.operation_timeout, 'clamped service target did not reach feedback')
            left.set_angle(target)
            wait_state(left, lambda s: list(s.angle_raw) == target,
                       args.operation_timeout, 'original target did not reach feedback')
            bad_modes = [0] * 13
            bad_modes[3] = 1
            expect_rejected(lambda: left.set_mode(bad_modes), 'must match')
            bad_modes = [0] * 13
            bad_modes[5] = 1
            expect_rejected(lambda: left.set_mode(bad_modes), 'must match')
            left.set_mode([0] * 13)
            require(list(left.get_state().angle_raw) == target, 'rejected writes changed angle feedback')
            report('service clamps angles with applied values; mode pairing remains strict')

            for topic, values in named_values.items():
                for kwargs in ({'zero_stamp': True}, {'offset_sec': -60}, {'offset_sec': 60}):
                    result = monitor.command(values, args.operation_timeout, topic=topic, **kwargs)
                    require(not result.accepted and 'timestamp' in result.message.lower(),
                            topic + ' bypassed timestamp validation')
                invalid = list(values)
                invalid[0] = 30000
                result = monitor.command(invalid, args.operation_timeout, topic=topic)
                if topic == 'angle_set':
                    expected = list(values)
                    expected[0] = ANGLE_MAX[0]
                    require(result.accepted and list(result.applied_values) == expected
                            and list(result.clamped) == [True] + [False] * 12,
                            'angle_set did not report clamped values')
                    wait_state(left, lambda s: list(s.angle_raw) == expected,
                               args.operation_timeout, 'clamped topic target did not reach feedback')
                    left.set_angle(target)
                    wait_state(left, lambda s: list(s.angle_raw) == target,
                               args.operation_timeout, 'target restore did not reach feedback')
                else:
                    require(not result.accepted and 'outside' in result.message.lower(),
                            topic + ' bypassed command limits')
            result = monitor.command(bad_modes, args.operation_timeout, topic='mode_set')
            require(not result.accepted and 'must match' in result.message.lower(),
                    'mode_set bypassed mode pairing')
            require(list(left.get_state().angle_raw) == target, 'rejected named commands changed feedback')
            report('named commands preserve timestamps, limits and mode-pair checks')

            outside = [-1] * 13
            outside[3] = -32768
            result = monitor.command(outside, args.operation_timeout)
            expected = outside.copy()
            expected[3] = ANGLE_MIN[3]
            require(result.accepted and list(result.applied_values) == expected
                    and result.clamped[3], 'unified command did not clamp negative splay')
            left.set_angle(target)
            wait_state(left, lambda s: list(s.angle_raw) == target,
                       args.operation_timeout, 'unified target restore did not reach feedback')

            for label, kwargs in [
                ('expired', {'offset_sec': -60}),
                ('future', {'offset_sec': 60}),
                ('missing', {'zero_stamp': True}),
            ]:
                result = monitor.command(original_left, args.operation_timeout, **kwargs)
                require(not result.accepted and 'timestamp' in result.message.lower(),
                        label + ' topic command was not rejected for timestamp')
                require(list(left.get_state().angle_raw) == target,
                        label + ' topic command changed feedback')
            report('expired, future, and unstamped topic commands return explicit rejection')

            # Capture immediately before enable, then publish immediately after
            # its response. The stamp is recent (< default 0.5 s age limit), but
            # belongs to the previous command-enable interval. Re-enabling must
            # not permit queued commands from before that transition to execute.
            left.set_enabled(False)
            before_enable_ns = monitor.node.get_clock().now().nanoseconds
            left.set_enabled(True)
            result = monitor.command(original_left, args.operation_timeout,
                                     stamp_ns_override=before_enable_ns)
            require(not result.accepted and
                    'timestamp predates the current enable' in result.message.lower(),
                    'pre-enable topic command was not rejected by enable epoch: ' + result.message)
            require(list(left.get_state().angle_raw) == target,
                    'pre-enable topic command changed feedback after re-enable')
            report('recent topic command predating the current enable interval is rejected')

            left.set_enabled(False)
            before_enable_ns = monitor.node.get_clock().now().nanoseconds
            left.set_enabled(True)
            result = monitor.command(original_left, args.operation_timeout, topic='angle_set',
                                     stamp_ns_override=before_enable_ns)
            require(not result.accepted and 'predates' in result.message.lower(),
                    'angle_set bypassed the current enable epoch')
            result = monitor.command(original_left, args.operation_timeout, topic='angle_set')
            require(result.accepted, 'fresh angle_set rejected: ' + result.message)
            monitor.wait(lambda: list(monitor.feedback['left', 'angle_actual'].values) == original_left,
                         args.operation_timeout, 'angle_actual did not reflect angle_set')
            require(list(right.get_state().angle_raw) == original_right, 'angle_set crossed namespaces')
            report('angle_set respects enable epoch, updates angle_actual and preserves right hand isolation')

            result = monitor.command(original_left, args.operation_timeout)
            require(result.accepted, 'fresh topic command rejected: ' + result.message)
            wait_state(left, lambda s: fresh(s) and list(s.angle_raw) == original_left,
                       args.operation_timeout, 'accepted topic command did not restore initial angle')
            require(list(right.get_state().angle_raw) == original_right, 'topic command crossed namespaces')
            report('fresh topic command returns matching CommandResult and restores initial angle')

            left.stop()
            require(not left.get_state().enabled, 'stop did not disable command gate')
            expect_rejected(lambda: left.set_angle(target), 'disabled')
            left.reconnect()
            restored = wait_state(left, fresh, args.discovery_timeout, 'reconnect feedback did not recover')
            require(not restored.enabled, 'reconnect unexpectedly enabled commands')
            require(list(restored.angle_raw) == original_left, 'reconnect replayed an old command')
            before_clear = stamp_ns(restored.error_stamp)
            left.clear_errors()
            cleared = wait_state(left, lambda s: fresh(s) and stamp_ns(s.error_stamp) > before_clear,
                                 args.operation_timeout, 'clear_errors did not reacquire error feedback')
            require(all(code == 0 for code in cleared.error_code), 'serial error feedback not cleared')
            require(not cleared.enabled, 'clear_errors unexpectedly enabled commands')
            left.clear_stop()
            require(not left.get_state().enabled, 'clear_stop unexpectedly enabled commands')
            report('stop disables; reconnect stays disabled; clear_errors reacquires fresh error feedback')

        finally:
            failures = []
            for hand in verified_clients:
                try:
                    hand.set_enabled(False)
                except Exception as exc:
                    failures.append('{}: {}'.format(hand.namespace, exc))
            if failures:
                raise HandError('serial cleanup failed: ' + '; '.join(failures))
        require(not left.get_state().enabled and not right.get_state().enabled,
                'cleanup left a serial command gate enabled')
        report('cleanup left both command gates disabled')

