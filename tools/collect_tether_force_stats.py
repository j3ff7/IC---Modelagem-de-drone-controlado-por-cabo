#!/usr/bin/env python3
import argparse
import json
import math
import re
import subprocess


def parse_vector3d_stream(text):
    values = []
    current = {}
    for line in text.splitlines():
        match = re.match(r'\s*([xyz]):\s*([-+0-9.eE]+)', line)
        if not match:
            continue
        current[match.group(1)] = float(match.group(2))
        if {'x', 'y'} <= current.keys():
            values.append((current['x'], current['y'], current.get('z', 0.0)))
            current = {}
    return values


def rms(items):
    if not items:
        return None
    return math.sqrt(sum(value * value for value in items) / len(items))


def main():
    parser = argparse.ArgumentParser(description='Collect /cabo/conexao/stats metrics.')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--topic', default='/cabo/conexao/stats')
    args = parser.parse_args()

    cmd = [
        'gz',
        'topic',
        '-e',
        '-t',
        args.topic,
        '-n',
        str(args.samples),
    ]
    result = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=args.timeout,
        check=False,
    )
    values = parse_vector3d_stream(result.stdout)
    errors = [item[0] for item in values]
    forces = [item[1] for item in values]
    saturated = [item[2] >= 0.5 for item in values]
    output = {
        'topic': args.topic,
        'requested_samples': args.samples,
        'samples': len(values),
        'error_rms_m': rms(errors),
        'error_max_m': max(errors) if errors else None,
        'force_rms_n': rms(forces),
        'force_max_n': max(forces) if forces else None,
        'saturation_fraction': (sum(saturated) / len(saturated)) if saturated else None,
        'gz_topic_returncode': result.returncode,
        'stderr': result.stderr.strip(),
    }
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
