"""Summarize live decision evidence without equating process exit with mission success.

Accept a trace.jsonl or a run.tar.gz. Archives are streamed without extracting files.
This intentionally reports map counters as counters, not ground-truth coverage.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import tarfile


def read_rows(path):
    if str(path).endswith('.tar.gz'):
        with tarfile.open(path, 'r|gz') as archive:
            found = False
            for member in archive:
                if member.name.endswith('/trace.jsonl') and '/replay' not in member.name:
                    if found:
                        raise ValueError('archive contains multiple live traces; select one explicitly')
                    found = True
                    for line in archive.extractfile(member):
                        if line.strip():
                            yield json.loads(line)
            if not found:
                raise ValueError('no live trace in archive')
    else:
        with open(path) as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)


def summarize(rows):
    agents = defaultdict(list)
    for row in rows:
        agents[int(row['agent'])].append(row)
    report = {'scope': 'decision-trace diagnostics; no ground-truth coverage or completion inference', 'agents': {}}
    for agent, rows in sorted(agents.items()):
        previous = None
        switches = 0
        goals = Counter()
        reasons = Counter()
        for row in rows:
            candidates = row.get('candidates', [])
            chosen = next((c for c in candidates if c.get('index') == row.get('chosen')), None)
            if chosen and chosen.get('goal_xy_m'):
                goal = tuple(chosen['goal_xy_m'])
                goals[tuple(round(x, 1) for x in goal)] += 1
                if previous is not None and math.dist(previous, goal) > 0.5:
                    switches += 1
                previous = goal
            states = row.get('planner', {}).get('agents', [])
            if 0 <= agent < len(states):
                reasons[states[agent].get('why', 'missing')] += 1
        vision = rows[-1].get('vision', {})
        report['agents'][str(agent)] = {
            'decisions': len(rows), 'last_sim_s': rows[-1]['t_s'],
            'commands': dict(Counter(r.get('command', {}).get('kind', 'missing') for r in rows)),
            'empty_candidate_decisions': sum(not r.get('candidates') for r in rows),
            'goal_switches_over_0_5m': switches,
            'most_repeated_rounded_goals': [{'xy_m': list(k), 'decisions': v} for k, v in goals.most_common(5)],
            'allocation_reasons': dict(reasons),
            'any_egress_found': any(r.get('vision', {}).get('egress_found', False) for r in rows),
            'final_map_counters': {k: vision.get(k) for k in ('free_cells', 'occupied_cells', 'unknown_cells', 'keyframes_integrated')},
            'final_communication_bytes': rows[-1].get('planner', {}).get('bytes'),
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(read_rows(args.input))
    result['source'] = str(args.input)
    with args.input.open('rb') as stream:
        result['sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
