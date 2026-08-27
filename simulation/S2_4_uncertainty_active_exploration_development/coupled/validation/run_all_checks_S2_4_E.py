#!/usr/bin/env python3
"""Aggregate static/offline checks for the S2.4-E coupled candidate."""
from __future__ import annotations
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
for cmd in (
    [sys.executable, str(ROOT / "tools" / "run_all_checks.py")],
    [sys.executable, str(ROOT / "coupled" / "validation" / "audit_S2_4_E_static.py")],
    [sys.executable, str(ROOT / "coupled" / "validation" / "audit_S2_4_E_literal_corridor_geometry.py")],
    [sys.executable, str(ROOT / "coupled" / "validation" / "audit_S2_4_E_scenario2_preflight.py")],
):
    print("\n$", " ".join(map(str, cmd)))
    p = subprocess.run(cmd, cwd=ROOT)
    if p.returncode:
        print("\nS2.4-E AGGREGATE STATIC/OFFLINE GATE: FAIL")
        raise SystemExit(p.returncode)
preflight = ROOT / "coupled" / "evidence" / "S2_4_E_GOAL_REQUIRES_SCAN_PREFLIGHT.json"
if not preflight.is_file():
    raise SystemExit("Missing goal-requires-scan recorded preflight evidence.")
data=json.loads(preflight.read_text())
assert data.get("scenario")=="GOAL_REQUIRES_SCAN"
assert data.get("pass") is True
assert data.get("deterministic_repeat_match") is True
assert data.get("unsafe_accepted_candidates")==0
assert data.get("accepted_candidates",0)>=1
print("\nGoal-requires-scan recorded preflight: PASS")
print(f"Snapshots: {data['snapshots']} | accepted candidates: {data['accepted_candidates']} | unsafe: 0")
print("\nS2.4-E AGGREGATE STATIC/OFFLINE GATE: PASS")
print("Literal-corridor static geometry contract: PASS")
print("Legacy v0.3.0 recorded-map selector regression: PASS")
print("MATLAB coupled literal-corridor mission: PENDING LOCAL MATLAB RUN")
