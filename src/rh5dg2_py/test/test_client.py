"""Failure and boundary behavior without ROS, serial ports, or hardware."""

import threading
from types import SimpleNamespace

import pytest

from rh5dg2_py import (
    CommandKind, HandClient, HandError, HandRejectedError, HandTimeoutError,
    validate_values,
)
from rh5dg2_py.cli import parse_args


class FakeFuture:
    def __init__(self, response=None, done=True, error=None):
        self.response = response
        self.completed = done
        self.error = error
        self.cancelled = False

    def done(self):
        return self.completed

    def cancel(self):
        self.cancelled = True

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class FakeService:
    srv_type = SimpleNamespace(Request=SimpleNamespace)

    def __init__(self, available=True, future=None):
        self.available = available
        self.future = future
        self.calls = []
        self.discovery_timeout = None
        self.removed = []

    def wait_for_service(self, timeout_sec):
        self.discovery_timeout = timeout_sec
        return self.available

    def call_async(self, request):
        self.calls.append(request)
        return self.future

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeExecutor:
    def __init__(self):
        self.response_timeout = None

    def spin_until_future_complete(self, future, timeout_sec):
        self.response_timeout = timeout_sec


def make_client(service):
    hand = HandClient.__new__(HandClient)
    hand.namespace = '/inspire/left'
    hand.timeout_sec = 2.0
    hand._lock = threading.RLock()
    hand._closed = False
    hand._clients = {'get_state': service, 'set_values': service, 'set_enabled': service}
    hand._executor = FakeExecutor()
    return hand


def test_absent_service_never_sends_request():
    service = FakeService(available=False)
    hand = make_client(service)
    with pytest.raises(HandTimeoutError, match='service unavailable'):
        hand.get_state()
    assert service.calls == []
    assert 0 < service.discovery_timeout <= 2.0


def test_discovery_and_response_share_one_deadline(monkeypatch):
    # Discovery consumes 1.2 seconds of the 2-second total budget.
    times = iter([10.0, 10.0, 11.2, 11.2])
    monkeypatch.setattr('rh5dg2_py.client.time.monotonic', lambda: next(times))
    state = object()
    service = FakeService(future=FakeFuture(SimpleNamespace(success=True, message='', state=state)))
    hand = make_client(service)
    assert hand.get_state() is state
    assert service.discovery_timeout == 2.0
    assert hand._executor.response_timeout == pytest.approx(0.8)


def test_timeout_cancels_pending_future_without_retry():
    future = FakeFuture(done=False)
    service = FakeService(future=future)
    hand = make_client(service)
    with pytest.raises(HandTimeoutError, match='may already have executed'):
        hand.set_speed([100] * 13)
    assert future.cancelled
    assert service.removed == [future]
    assert len(service.calls) == 1


def test_driver_rejection_is_not_success_or_zero_state():
    service = FakeService(future=FakeFuture(SimpleNamespace(success=False, message='stale feedback')))
    with pytest.raises(HandRejectedError, match='stale feedback'):
        make_client(service).get_state()


def test_future_exception_is_reported():
    service = FakeService(future=FakeFuture(error=RuntimeError('DDS failed')))
    with pytest.raises(HandError, match='DDS failed'):
        make_client(service).get_state()


def test_closed_client_never_sends():
    service = FakeService()
    hand = make_client(service)
    hand._closed = True
    with pytest.raises(HandError, match='closed'):
        hand.get_state()
    assert not service.calls


@pytest.mark.parametrize('values', [[0] * 6, [0] * 12, [0] * 14, [0.0] * 13, [True] * 13])
def test_invalid_values_rejected_before_call(values):
    service = FakeService()
    with pytest.raises(ValueError):
        make_client(service).set_angle(values)
    assert not service.calls


@pytest.mark.parametrize('kind', list(CommandKind))
def test_int16_bounds_apply_to_every_command(kind):
    assert validate_values(kind, [32767] * 13) == [32767] * 13
    assert validate_values(kind, [-32768] * 13) == [-32768] * 13
    with pytest.raises(ValueError):
        validate_values(kind, [32768] * 13)
    with pytest.raises(ValueError):
        validate_values(kind, [-32769] * 13)


@pytest.mark.parametrize('kind', [True, -1, 5, '0', 0.0, None])
def test_invalid_kind(kind):
    with pytest.raises(ValueError):
        validate_values(kind, [0] * 13)


def test_hardware_range_rejection_comes_from_driver():
    response = SimpleNamespace(success=False, message='mode must be 0 or 1')
    service = FakeService(future=FakeFuture(response))
    with pytest.raises(HandRejectedError, match='mode must be 0 or 1'):
        make_client(service).set_mode([2] * 13)
    assert len(service.calls) == 1


def test_angle_sentinel_and_signed_values_preserved():
    values = [-1, -100, 0] + [1400] * 10
    assert validate_values(CommandKind.ANGLE, values) == values
    with pytest.raises(ValueError):
        validate_values(CommandKind.ANGLE, [32768] * 13)


def test_set_method_preserves_values_and_kind():
    response = SimpleNamespace(success=True, message='accepted')
    service = FakeService(future=FakeFuture(response))
    hand = make_client(service)
    assert hand.set_force([200] * 13) is response
    assert service.calls[0].kind == 2
    assert service.calls[0].values == [200] * 13


def test_enabled_requires_bool():
    service = FakeService()
    with pytest.raises(ValueError):
        make_client(service).set_enabled('false')
    assert not service.calls


def test_cli_defaults_to_read_and_rejects_six_channel_commands():
    assert parse_args([]).command == 'state'
    with pytest.raises(SystemExit) as exc:
        parse_args(['set-angle'] + ['1000'] * 6)
    assert exc.value.code == 2
    args = parse_args(['--namespace', '/inspire/right', 'raw', '1'] + ['100'] * 13)
    assert args.kind == CommandKind.SPEED
    assert args.namespace == '/inspire/right'


def test_close_only_shuts_down_owned_resources():
    actions = []
    service = FakeService()
    hand = make_client(service)
    node = SimpleNamespace(destroy_node=lambda: actions.append('destroy node'))
    hand._node = node
    hand._executor = SimpleNamespace(
        remove_node=lambda node: actions.append('remove node'),
        shutdown=lambda timeout_sec: actions.append('shutdown executor'),
    )
    hand._context = SimpleNamespace(
        ok=lambda: True,
        shutdown=lambda: actions.append('shutdown private context'),
    )
    hand.close()
    hand.close()
    assert actions == ['remove node', 'shutdown executor', 'destroy node', 'shutdown private context']
    assert not service.calls


def test_topic_command_ids_and_close():
    hand = make_client(FakeService())
    published = []
    hand._command_id = 0
    hand._publisher = SimpleNamespace(
        msg_type=lambda: SimpleNamespace(header=SimpleNamespace()),
        publish=published.append,
    )
    hand._node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'now')),
    )
    assert hand.publish_command(CommandKind.ANGLE, [-1] * 13) == 1
    assert hand.publish_command(CommandKind.SPEED, [100] * 13, command_id=42) == 42
    assert published[0].values == [-1] * 13
    assert published[1].kind == CommandKind.SPEED
    assert published[1].command_id == 42
    with pytest.raises(ValueError, match='uint64'):
        hand.publish_command(CommandKind.ANGLE, [0] * 13, command_id=-1)
    hand._closed = True
    with pytest.raises(HandError, match='closed'):
        hand.publish_command(CommandKind.ANGLE, [0] * 13)
    assert len(published) == 2
