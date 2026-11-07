#!/usr/bin/env python3
"""
Run the whole challenge headless over many layouts and print the referee's verdicts.

    python3 tools/run_trials.py --seeds 0-19
    python3 tools/run_trials.py --seeds 3,7 --count 8

Each seed gets a fresh simulator. Source the workspace first. Exits non-zero unless
every run passed.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time


def parse_seeds(text):
    seeds = []
    for part in text.split(','):
        if '-' in part:
            first, last = part.split('-')
            seeds.extend(range(int(first), int(last) + 1))
        else:
            seeds.append(int(part))
    return seeds


def run_seed(seed, count, timeout, keep_log):
    with tempfile.TemporaryDirectory() as folder:
        result_file = os.path.join(folder, 'result.json')
        command = ['ros2', 'launch', 'fastrobox_sim', 'competition.launch.py', 'gui:=false',
                   f'seed:={seed}', f'result_file:={result_file}']
        if count:
            command.append(f'box_count:={count}')
        log = open(keep_log, 'w') if keep_log else subprocess.DEVNULL
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline and process.poll() is None:
                if os.path.exists(result_file) and os.path.getsize(result_file) > 0:
                    time.sleep(0.2)
                    with open(result_file) as handle:
                        return json.load(handle)
                time.sleep(0.5)
            return None
        finally:
            for signum, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    break
                time.sleep(wait)
            if keep_log:
                log.close()


def main():
    parser = argparse.ArgumentParser(description='Run the challenge over many layouts.')
    parser.add_argument('--seeds', default='0-9', help='For example 0-19 or 3,7,11.')
    parser.add_argument('--count', type=int, default=None, help='Boxes; default 6 to 8.')
    parser.add_argument('--timeout', type=float, default=150.0, help='Wall seconds per run.')
    parser.add_argument('--log-dir', default=None, help='Keep each launch log here.')
    parser.add_argument('--json', default=None, help='Write all verdicts to this file.')
    args = parser.parse_args()

    verdicts = []
    print('seed  result  boxes  time_s  final_x  final_y  box_gap  wall_gap  illegal', flush=True)
    for seed in parse_seeds(args.seeds):
        keep_log = os.path.join(args.log_dir, f'seed_{seed}.log') if args.log_dir else None
        verdict = run_seed(seed, args.count, args.timeout, keep_log)
        if verdict is None:
            verdicts.append({'seed': seed, 'passed': False, 'finished': False})
            print(f'{seed:4}  NO RESULT within {args.timeout:.0f} s', flush=True)
            continue
        verdicts.append(verdict)
        print(f"{seed:4}  {'PASS' if verdict['passed'] else 'FAIL':6}  "
              f"{len(verdict['boxes']):5}  {verdict['time_s']:6}  {verdict['final_x']:7}  "
              f"{verdict['final_y']:7}  {verdict['min_box_gap']:7}  "
              f"{verdict['min_wall_gap']:8}  {verdict['illegal_boxes']:7}", flush=True)

    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(verdicts, handle, indent=2)
    passed = sum(1 for verdict in verdicts if verdict['passed'])
    print(f'{passed} of {len(verdicts)} runs passed', flush=True)
    return 0 if passed == len(verdicts) else 1


if __name__ == '__main__':
    sys.exit(main())
