"""Synchronous ROS 2 client with an isolated context and bounded requests."""

import math
from numbers import Integral
import threading
import time
import uuid

from .validation import CommandKind, validate_values


class HandError(RuntimeError):
    """ROS transport or driver failure."""


class HandTimeoutError(HandError):
    """The service did not appear or reply within the request deadline."""


class HandRejectedError(HandError):
    """The driver explicitly rejected a request."""


class HandClient:
    """Blocking hand API with a private ROS context and one timeout per call.

    Use with a context manager. The driver accepts commands by default after
    initial feedback; set_enabled controls its software gate when needed.
    Closing only releases ROS resources. Timed-out writes may have executed.
    """

    def __init__(self, namespace='/inspire/left', timeout_sec=2.0):
        timeout_sec = float(timeout_sec)
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('timeout_sec must be a finite positive number')
        if not isinstance(namespace, str) or not namespace.strip('/'):
            raise ValueError('namespace must identify a hand, e.g. /inspire/left')
        # Keep the package importable without ROS.
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rh5dg2_interfaces.msg import HandCommand
        from rh5dg2_interfaces.srv import GetState, SetEnabled, SetValues
        from std_srvs.srv import Trigger

        self.namespace = '/' + namespace.strip('/')
        self.timeout_sec = timeout_sec
        self._lock = threading.RLock()
        self._closed = False
        self._command_id = 0
        self._context = Context()
        self._node = None
        self._executor = None
        try:
            rclpy.init(args=[], context=self._context)
            self._node = rclpy.create_node(
                'rh5dg2_client_' + uuid.uuid4().hex[:12],
                namespace=self.namespace, context=self._context,
                use_global_arguments=False,
            )
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._clients = {
                name: self._node.create_client(service_type, name)
                for name, service_type in (
                    ('set_values', SetValues), ('get_state', GetState),
                    ('set_enabled', SetEnabled), ('stop', Trigger),
                    ('clear_stop', Trigger),
                    ('clear_errors', Trigger), ('reconnect', Trigger),
                )
            }
            self._publisher = self._node.create_publisher(HandCommand, 'command', 10)
        except Exception:
            self.close()
            raise

    def _ensure_open(self):
        if self._closed:
            raise HandError('HandClient is closed')

    def _call(self, name, **fields):
        with self._lock:
            self._ensure_open()
            deadline = time.monotonic() + self.timeout_sec
            client = self._clients[name]
            endpoint = self.namespace + '/' + name
            try:
                if not client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
                    raise HandTimeoutError(
                        f'service unavailable: {endpoint}; check driver, namespace and ROS_DOMAIN_ID')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise HandTimeoutError(f'service discovery timed out: {endpoint}')
                future = client.call_async(client.srv_type.Request(**fields))
                self._executor.spin_until_future_complete(
                    future, timeout_sec=max(0.0, deadline - time.monotonic()))
                if not future.done():
                    future.cancel()
                    client.remove_pending_request(future)
                    raise HandTimeoutError(
                        f'response timed out: {endpoint}; request may already have executed')
                response = future.result()
            except HandError:
                raise
            except Exception as exc:
                raise HandError(f'{endpoint}: {exc}') from exc
            if response is None:
                raise HandError(f'empty response: {endpoint}')
            if not response.success:
                raise HandRejectedError(f'{endpoint}: {response.message}')
            return response

    def get_state_response(self):
        """Return the full GetState response, raising on stale/unavailable data."""
        return self._call('get_state')

    def get_state(self):
        """Return the driver's complete HandState; angle_raw is in 0.1 degrees."""
        return self.get_state_response().state

    def set_values(self, kind, values):
        """Send 13 int16 values; the driver checks hardware ranges. Never enables."""
        values = validate_values(kind, values)
        return self._call('set_values', kind=int(kind), values=values)

    def set_angle(self, values):
        """Set 13 targets in 0.1 degrees; -1 skips a joint. Returns applied_values."""
        return self.set_values(CommandKind.ANGLE, values)

    def set_speed(self, values):
        """Set 13 speed registers, each in [0, 2500]."""
        return self.set_values(CommandKind.SPEED, values)

    def set_force(self, values):
        """Set 13 force registers, each in [0, 1000]."""
        return self.set_values(CommandKind.FORCE, values)

    def set_current_limit(self, values):
        """Set 13 current-limit registers, each in [0, 1500]."""
        return self.set_values(CommandKind.CURRENT_LIMIT, values)

    def set_mode(self, values):
        """Set 13 mode registers, each 0 or 1; semantics follow the vendor manual."""
        return self.set_values(CommandKind.MODE, values)

    def set_enabled(self, enabled):
        """Change software command acceptance only; no device register is written."""
        if not isinstance(enabled, bool):
            raise ValueError('enabled must be bool')
        return self._call('set_enabled', enabled=enabled)

    def stop(self):
        """Explicitly set emergency stop (1301=1) and disable commands."""
        return self._call('stop')

    def clear_stop(self):
        """Explicitly clear emergency stop (1301=0); commands remain disabled."""
        return self._call('clear_stop')

    def clear_errors(self):
        """Explicitly request clearing device errors; does not enable motion."""
        return self._call('clear_errors')

    def reconnect(self):
        """Explicitly reconnect; the driver remains disabled and drops old commands."""
        return self._call('reconnect')

    def publish_command(self, kind, values, command_id=None):
        """Publish and return command ID; use command_result for acknowledgement."""
        values = validate_values(kind, values)
        if command_id is not None and (
            isinstance(command_id, bool) or not isinstance(command_id, Integral)
            or not 0 <= command_id < 2 ** 64
        ):
            raise ValueError('command_id must be a uint64 integer')
        with self._lock:
            self._ensure_open()
            if command_id is None:
                self._command_id = (self._command_id + 1) % (2 ** 64)
                command_id = self._command_id
            message = self._publisher.msg_type()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.command_id = int(command_id)
            message.kind = int(kind)
            message.values = values
            self._publisher.publish(message)
            return int(command_id)

    def close(self):
        """Release local ROS resources. Does not send any command to the hand."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._executor is not None:
                if self._node is not None:
                    self._executor.remove_node(self._node)
                self._executor.shutdown(timeout_sec=0.0)
            if self._node is not None:
                self._node.destroy_node()
            if self._context.ok():
                self._context.shutdown()

    def __enter__(self):
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
