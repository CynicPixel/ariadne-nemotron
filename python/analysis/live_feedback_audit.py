"""Audit actual live motion and command age from a run archive, independently of exit status.

Movement is measured from recorded ground-truth body trajectories. Command age is
applied simulation time minus decision stamp, using simulator-side mover records.
This is a feedback-loop diagnostic, not a coverage, route or task-completion score.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import tarfile


def percentile(values, q):
    if not values:
        return None
    a = sorted(values)
    i = (len(a) - 1) * q
    lo, hi = math.floor(i), math.ceil(i)
    return a[lo] + (a[hi] - a[lo]) * (i - lo)


def audit(path):
    gt, movers = defaultdict(list), defaultdict(list)
    horizon = None
    result_files = 0
    with tarfile.open(path, 'r|gz') as archive:
        for member in archive:
            if not member.isfile() or '/replay' in member.name:
                continue
            name = member.name
            body = re.search(r'-agent(\d+)-[^/]+/raw/groundtruth_body\.tum$', name)
            mover = re.search(r'/mover_agent_(\d+)\.jsonl$', name)
            if name.endswith('/results.yaml'):
                content = archive.extractfile(member).read().decode()
                if not re.search(r'^mode: live-demo\s*$', content, re.M):
                    continue
                result_files += 1
                match = re.search(r'^end_s: ([0-9.]+)\s*$', content, re.M)
                if match:
                    horizon = float(match[1])
            elif body:
                gt[int(body[1])]  # retain agents with an empty recording
                for line in archive.extractfile(member):
                    if line.strip() and not line.lstrip().startswith(b'#'):
                        v = list(map(float, line.split()))
                        if len(v) != 8 or not all(math.isfinite(x) for x in v):
                            raise ValueError('invalid ground-truth pose')
                        gt[int(body[1])].append(v)
            elif mover:
                movers[int(mover[1])]
                for line in archive.extractfile(member):
                    if line.strip():
                        movers[int(mover[1])].append(json.loads(line))
    if result_files != 1 or horizon is None:
        raise ValueError('expected exactly one live-demo results file with end_s')
    result = {'scope': __doc__.split('\n\n')[1].strip(), 'end_s': horizon, 'agents': {}}
    for agent in sorted(gt.keys() | movers.keys()):
        poses = sorted((v for v in gt[agent] if 0 <= v[0] <= horizon), key=lambda v: v[0])
        travel = sum(math.dist(a[1:4], b[1:4]) for a, b in zip(poses, poses[1:])) if len(poses) > 1 else None
        commands = [r for r in movers[agent] if r.get('cmd_stamp_ns', -1) >= 0]
        inside = [r for r in commands if 0 <= r['applied_ns'] <= horizon * 1e9]
        ages = [(r['applied_ns'] - r['cmd_stamp_ns']) * 1e-9 for r in inside]
        result['agents'][str(agent)] = {
            'gt_samples': len(poses), 'gt_first_s': poses[0][0] if poses else None,
            'gt_last_s': poses[-1][0] if poses else None, 'gt_translation_path_m': travel,
            'gt_net_displacement_m': math.dist(poses[0][1:4], poses[-1][1:4]) if len(poses) > 1 else None,
            'applied_commands_within_horizon': len(inside),
            'applied_commands_after_horizon': sum(r['applied_ns'] > horizon * 1e9 for r in commands),
            'first_command_applied_s': min((r['applied_ns'] * 1e-9 for r in inside), default=None),
            'negative_command_age_count': sum(a < 0 for a in ages),
            'command_age_sim_s': {'p50': percentile(ages, .5), 'p95': percentile(ages, .95),
                                  'max': max(ages, default=None), 'samples': len(ages)},
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.archive)
    result['source'] = str(args.archive)
    with args.archive.open('rb') as stream:
        result['sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
