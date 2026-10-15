"""Small explicit CLI. With no subcommand it only reads state."""

import argparse
import json
import sys

from .client import HandClient, HandError
from .validation import CommandKind, validate_values


VALUE_COMMANDS = {
    'set-angle': CommandKind.ANGLE,
    'set-speed': CommandKind.SPEED,
    'set-force': CommandKind.FORCE,
    'set-current-limit': CommandKind.CURRENT_LIMIT,
    'set-mode': CommandKind.MODE,
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='RH5DG2 ROS 2 client; default operation only reads state.',
        epilog='Put --namespace and --timeout before the subcommand. '
               'Angle values are raw 0.1 degree registers; -1 skips a joint.',
    )
    parser.add_argument('--namespace', default='/inspire/left')
    parser.add_argument('--timeout', type=float, default=2.0,
                        help='total service discovery/response deadline in seconds')
    commands = parser.add_subparsers(dest='command')
    commands.add_parser('state', help='read feedback without enabling movement')
    commands.add_parser('enable', help='explicitly arm driver command acceptance')
    commands.add_parser('disable', help='disable new commands without changing device targets')
    commands.add_parser('stop', help='explicitly request emergency stop and disable commands')
    commands.add_parser('clear-stop', help='explicitly clear emergency stop; commands stay disabled')
    commands.add_parser('clear-errors', help='explicitly clear device errors')
    commands.add_parser('reconnect', help='reopen connection and remain disabled')
    for name, kind in VALUE_COMMANDS.items():
        command = commands.add_parser(name, help=f'write 13 {kind.name.lower()} registers')
        command.add_argument('values', nargs='+', type=int, metavar='VALUE')
    raw = commands.add_parser('raw', help='write 13 values using a command kind (not an arbitrary address)')
    raw.add_argument('kind', type=int, choices=[int(kind) for kind in CommandKind])
    raw.add_argument('values', nargs='+', type=int, metavar='VALUE')
    args = parser.parse_args(argv)
    args.command = args.command or 'state'
    if args.command in VALUE_COMMANDS or args.command == 'raw':
        if args.command in VALUE_COMMANDS:
            args.kind = VALUE_COMMANDS[args.command]
        try:
            args.values = validate_values(args.kind, args.values)
        except ValueError as exc:
            parser.error(str(exc))
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        with HandClient(namespace=args.namespace, timeout_sec=args.timeout) as hand:
            if args.command == 'state':
                from rosidl_runtime_py.convert import message_to_ordereddict
                print(json.dumps(message_to_ordereddict(hand.get_state()), indent=2))
                return 0
            if args.command in VALUE_COMMANDS or args.command == 'raw':
                result = hand.set_values(args.kind, args.values)
            elif args.command in ('enable', 'disable'):
                result = hand.set_enabled(args.command == 'enable')
            else:
                result = getattr(hand, args.command.replace('-', '_'))()
            print(result.message or 'OK')
            return 0
    except (HandError, ValueError) as exc:
        print(f'rh5dg2: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
