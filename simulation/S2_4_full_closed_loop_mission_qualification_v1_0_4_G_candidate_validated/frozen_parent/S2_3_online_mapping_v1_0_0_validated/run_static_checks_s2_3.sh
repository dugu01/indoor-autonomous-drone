#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 audit_S2_3_candidate.py
python3 audit_truth_isolation_S2_3.py .
python3 matlab_source_sanity.py .
python3 python_tests/test_s23_mechanisms.py
python3 python_tests/release_end_to_end_backtest.py --json RELEASE_END_TO_END_BACKTEST_STATIC.json
