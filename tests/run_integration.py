#!/usr/bin/env python3
"""Run serial ROS checks over internal PTYs with bounded process cleanup.

No hardware path or transport override is accepted. Source this workspace first,
or use scripts/test.sh to prepare the system Humble environment automatically.
"""

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time


WORKSPACE = Path(__file__).resolve().parents[1]


@contextmanager
def reserve_domain():
    """Avoid concurrent runners using the same localhost DDS domain.

    Serial tests reserve a localhost domain in 201..229, within Linux's
    supported DDS port range.
    Lock files remain in /tmp so concurrent processes always lock the same inode.
    """
    domains = list(range(201, 230))
    random.SystemRandom().shuffle(domains)
    for domain in domains:
        path = Path(tempfile.gettempdir()) / f'rh5dg2-test-{os.getuid()}-domain-{domain}.lock'
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(descriptor)
            continue
        try:
            yield domain
        finally:
            os.close(descriptor)
        return
    raise RuntimeError('All integration-test ROS domains are busy; try again after another run finishes')


class Child:
    def __init__(self, name, command, environment, log_path):
        self.name = name
        self.log_path = log_path
        with log_path.open('w', encoding='utf-8') as output:
            self.process = subprocess.Popen(
                command, cwd=WORKSPACE, env=environment,
                stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        self.groups = {self.process.pid}

    def capture_groups(self):
        """Track nested sessions too: the PTY test starts its own driver session."""
        pending = [self.process.pid]
        seen = set()
        while pending:
            pid = pending.pop()
            if pid in seen:
                continue
            seen.add(pid)
            try:
                group = os.getpgid(pid)
                if group != os.getpgrp():
                    self.groups.add(group)
                children = Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
                pending.extend(int(child) for child in children)
            except (ProcessLookupError, FileNotFoundError):
                pass

    @staticmethod
    def _alive(group):
        try:
            os.killpg(group, 0)
            return True
        except ProcessLookupError:
            return False

    def stop(self):
        self.capture_groups()
        # SIGINT lets launch and the test's finally blocks clean up first. The
        # additional groups cover a stuck test or an already-exited launcher.
        for signum, grace in ((signal.SIGINT, 4.0), (signal.SIGTERM, 1.0), (signal.SIGKILL, 1.0)):
            for group in self.groups:
                try:
                    os.killpg(group, signum)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                self.process.poll()  # Reap a finished direct child.
                if not any(self._alive(group) for group in self.groups):
                    break
                time.sleep(0.05)
            if not any(self._alive(group) for group in self.groups):
                break
        try:
            self.process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            print(f'WARNING: {self.name} did not exit after SIGKILL; inspect {self.log_path}',
                  file=sys.stderr, flush=True)


def print_log(child, tail=None):
    lines = child.log_path.read_text(encoding='utf-8', errors='replace').splitlines()
    if tail is not None:
        lines = lines[-tail:]
    if lines:
        print(f'--- {child.name} ---', flush=True)
        print('\n'.join(lines), flush=True)


def run_check(child, timeout, all_children):
    deadline = time.monotonic() + timeout
    while True:
        for process in all_children:
            process.capture_groups()
        result = child.process.poll()
        if result is not None:
            if result != 0:
                raise RuntimeError(f'{child.name} failed with exit status {result}; see {child.log_path}')
            print_log(child)
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f'{child.name} exceeded {timeout:g} seconds; see {child.log_path}')
        time.sleep(0.05)


def validate_workspace():
    from ament_index_python.packages import get_package_prefix

    install = (WORKSPACE / 'install').resolve()
    if not (install / 'setup.bash').is_file():
        raise RuntimeError('Workspace is not built; run scripts/build.sh first')
    for package in ('rh5dg2_driver', 'rh5dg2_bringup', 'rh5dg2_interfaces', 'rh5dg2_py'):
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(install):
            raise RuntimeError(f'{package} resolved outside this workspace: {prefix}; use scripts/test.sh')
    for filename in ('check_serial_interfaces.py', 'test_serial_ros.py'):
        if not (WORKSPACE / 'tests' / filename).is_file():
            raise RuntimeError(f'Missing integration check: tests/{filename}')


def main():
    if len(sys.argv) != 1:
        print('This offline runner accepts no hardware paths or extra arguments.', file=sys.stderr)
        return 2
    children = []
    log_dir = None
    success = False
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        validate_workspace()
        run_name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + f'-{os.getpid()}'
        log_dir = WORKSPACE / 'log' / 'integration' / run_name
        log_dir.mkdir(parents=True, exist_ok=False)
        environment = dict(os.environ)
        # Inherited DDS discovery settings must not redirect the local test
        # to another machine or override localhost discovery.
        for name in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                     'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
            environment.pop(name, None)
        environment.update(ROS_LOCALHOST_ONLY='1',
                           ROS_LOG_DIR=str(log_dir / 'ros'), PYTHONNOUSERSITE='1',
                           PYTHONUNBUFFERED='1')
        print(f'Serial integration: internally owned PTYs; logs: {log_dir}', flush=True)
        (log_dir / 'run.txt').write_text(
            'localhost_only=1\ntransport=serial; internally generated PTYs only\n', encoding='utf-8')
        serial = Child('PTY serial ROS checks', [
            sys.executable, str(WORKSPACE / 'tests/test_serial_ros.py'),
        ], environment, log_dir / 'serial_test.log')
        children.append(serial)
        # test_serial_ros has its own 60-second deadline and cleanup; this
        # outer deadline also protects against failures before its timer starts.
        run_check(serial, 75.0, children)
        success = True
    except KeyboardInterrupt:
        print('Integration interrupted; stopping child process groups.', file=sys.stderr, flush=True)
        return 130
    except Exception as error:
        print(f'FAIL integration: {error}', file=sys.stderr, flush=True)
        for child in children:
            print_log(child, tail=60)
        return 1
    finally:
        # Ignore repeated termination requests only during bounded cleanup.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            for child in reversed(children):
                child.stop()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
        if log_dir is not None:
            print(f'Integration logs retained at {log_dir}', flush=True)
    if success:
        print('PASS: serial startup, fault handling, dual-hand interfaces and angle demo; no physical device opened.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
