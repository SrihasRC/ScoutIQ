#!/usr/bin/env python3
"""CLI utility to save current map to map.pgm and map.yaml using nav2_map_server."""
import argparse
import os
import shutil
import subprocess
import sys


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Save map from /map topic to map.pgm and map.yaml using nav2_map_server."
    )
    parser.add_argument(
        'map_target',
        nargs='?',
        default='map',
        help="Target output directory or map file prefix (default: ./map -> map.pgm, map.yaml)"
    )
    parser.add_argument(
        '-t', '--topic',
        default='/map',
        help="Map topic to subscribe to (default: /map)"
    )
    parser.add_argument(
        '--occ',
        type=float,
        default=0.65,
        help="Occupied threshold (default: 0.65)"
    )
    parser.add_argument(
        '--free',
        type=float,
        default=0.25,
        help="Free threshold (default: 0.25)"
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help="Timeout in seconds to wait for map (default: 10.0)"
    )
    return parser.parse_args(argv)


def resolve_prefix(target: str) -> str:
    """Resolve target path to a base prefix without .pgm or .yaml extension."""
    had_trailing_slash = target.endswith('/') or target.endswith('\\') or target.endswith(os.sep)
    abs_target = os.path.abspath(target)

    # If target is an existing directory or user specified a trailing slash:
    if os.path.isdir(abs_target) or had_trailing_slash:
        os.makedirs(abs_target, exist_ok=True)
        return os.path.join(abs_target, 'map')

    # If target ends with .yaml or .pgm, strip extension
    if abs_target.endswith('.yaml'):
        abs_target = abs_target[:-5]
    elif abs_target.endswith('.pgm'):
        abs_target = abs_target[:-4]

    parent_dir = os.path.dirname(abs_target)
    if parent_dir and not os.path.exists(parent_dir):
        os.makedirs(parent_dir, exist_ok=True)

    return abs_target


def save_map(prefix: str, topic: str = '/map', occ: float = 0.65, free: float = 0.25, timeout: float = 10.0) -> bool:
    """Run nav2_map_server map_saver_cli to save map."""
    # Look for map_saver_cli binary in standard ROS2 locations
    cli_binary = '/opt/ros/humble/lib/nav2_map_server/map_saver_cli'
    if not (os.path.isfile(cli_binary) and os.access(cli_binary, os.X_OK)):
        cli_binary = shutil.which('map_saver_cli')

    if cli_binary and os.path.isfile(cli_binary) and os.access(cli_binary, os.X_OK):
        cmd = [
            cli_binary,
            '-t', topic,
            '-f', prefix,
            '--occ', str(occ),
            '--free', str(free),
            '--fmt', 'pgm',
            '--ros-args', '-p', 'use_sim_time:=true'
        ]
    else:
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', topic,
            '-f', prefix,
            '--occ', str(occ),
            '--free', str(free),
            '--fmt', 'pgm',
            '--ros-args', '-p', 'use_sim_time:=true'
        ]

    print(f"Running map saver: {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, timeout=timeout, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        if result.returncode != 0:
            print(f"ERROR: map_saver_cli exited with code {result.returncode}", file=sys.stderr)
            return False
    except subprocess.TimeoutExpired:
        print(f"ERROR: map_saver_cli timed out after {timeout} seconds", file=sys.stderr)
        return False
    except Exception as e:
        print(f"ERROR: Failed to run map_saver_cli: {e}", file=sys.stderr)
        return False

    pgm_path = f"{prefix}.pgm"
    yaml_path = f"{prefix}.yaml"

    if os.path.exists(pgm_path) and os.path.exists(yaml_path):
        if os.path.getsize(pgm_path) > 0 and os.path.getsize(yaml_path) > 0:
            print(f"SUCCESS: Map saved to {pgm_path} and {yaml_path}")
            return True
        else:
            print(f"ERROR: Saved map files are empty ({pgm_path}, {yaml_path})", file=sys.stderr)
            return False
    else:
        print(f"ERROR: Expected map files were not created: {pgm_path}, {yaml_path}", file=sys.stderr)
        return False


def main(argv=None):
    args = parse_args(argv)
    prefix = resolve_prefix(args.map_target)
    success = save_map(
        prefix=prefix,
        topic=args.topic,
        occ=args.occ,
        free=args.free,
        timeout=args.timeout
    )
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
