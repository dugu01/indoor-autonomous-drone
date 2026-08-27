from __future__ import annotations

import argparse
import json
from pathlib import Path

from s2_4_recorded_shadow_replay import replay_once


def main() -> int:
    parser = argparse.ArgumentParser(description='Run deterministic S2.4 shadow replay over compatible final S2.3 MAT traces.')
    parser.add_argument('mat_files', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, default=Path('evidence/S2_4_AD_RECORDED_REPLAY_MATRIX.json'))
    args = parser.parse_args()

    rows = []
    for mat_file in args.mat_files:
        row = {'file': str(mat_file), 'pass': False, 'error': ''}
        try:
            first = replay_once(mat_file)
            second = replay_once(mat_file)
            exact = (
                first['replay_digest'] == second['replay_digest']
                and first['uncertainty']['digest'] == second['uncertainty']['digest']
            )
            row.update({
                'scenario': first['scenario'],
                'snapshots': first['snapshots'],
                'deterministic_repeat': exact,
                'incremental_frontier_equals_full': first['incremental_frontier_equals_full'],
                'accepted_candidates': first['accepted_candidates'],
                'unsafe_accepted_candidates': first['unsafe_accepted_candidates'],
                'tracks_created': first['tracks_created'],
                'replay_digest': first['replay_digest'],
                'uncertainty_digest': first['uncertainty']['digest'],
                'pass': bool(first['pass'] and exact),
            })
        except Exception as exc:  # keep matrix catalogue complete
            row['error'] = f'{type(exc).__name__}: {exc}'
        rows.append(row)

    result = {
        'schema': 'S2_4_AD_RECORDED_REPLAY_MATRIX_V1',
        'count': len(rows),
        'passed': sum(bool(row['pass']) for row in rows),
        'rows': rows,
    }
    result['pass'] = result['passed'] == result['count']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    for row in rows:
        print(f"{row.get('scenario', row['file']):32s} {'PASS' if row['pass'] else 'FAIL'} {row['error']}")
    print(f"RECORDED REPLAY MATRIX: {result['passed']}/{result['count']} {'PASS' if result['pass'] else 'FAIL'}")
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
