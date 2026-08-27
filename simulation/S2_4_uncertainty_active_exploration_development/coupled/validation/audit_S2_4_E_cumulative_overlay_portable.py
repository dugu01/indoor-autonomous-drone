#!/usr/bin/env python3
"""Portable audit for the cumulative S2.4-E v0.3.3 overlay.

This check is intentionally runnable on the overlay archive by itself. It checks
package completeness, source-level mission-request/truth-isolation contracts,
Python syntax, and literal-corridor geometry. It does NOT replace the full
project gate, because the archive intentionally excludes frozen_parent/, tools/,
python_tests/, s2_4_shadow/, and the recorded S2.3 MAT trace.
"""
from __future__ import annotations
import json
import pathlib
import py_compile
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
C = ROOT / "coupled"

required = [
    C / "mission" / "init_S2_4_E_config.m",
    C / "mission" / "exploration_request_S2_4.m",
    C / "mission" / "plan_active_exploration_segment_S2_4.m",
    C / "mission" / "mission_lifecycle_manager_S2_4.m",
    C / "mission" / "run_S2_4_coupled.m",
    C / "execution" / "build_execution_grid_S2_4.m",
    C / "execution" / "project_uncertainty_2d_S2_4.m",
    C / "execution" / "validate_exploration_request_S2_4.m",
    C / "execution" / "summarize_viewpoint_decision_S2_4.m",
    C / "scenarios" / "scenario_S2_4.m",
    C / "scenarios" / "make_literal_competing_corridors_S2_4.m",
    C / "scenarios" / "literal_competing_corridors_geometry.json",
    C / "validation" / "validate_S2_4_E_milestone_1.m",
    C / "validation" / "validate_S2_4_E_milestone_2.m",
    C / "validation" / "validate_S2_4_E_competing_corridors_multiseed.m",
    C / "validation" / "test_S2_4_E_competing_decision_contract.m",
    C / "validation" / "test_S2_4_E_literal_corridor_geometry_contract.m",
    C / "validation" / "test_S2_4_E_request_contracts.m",
    C / "validation" / "validate_S2_4_E_all.m",
    C / "validation" / "audit_S2_4_E_literal_corridor_geometry.py",
    C / "validation" / "audit_S2_4_E_scenario2_preflight.py",
    C / "validation" / "audit_S2_4_E_static.py",
    C / "validation" / "run_all_checks_S2_4_E.py",
    ROOT / "setup_S2_4_E_path.m",
]

errors: list[str] = []
for p in required:
    if not p.is_file():
        errors.append(f"missing: {p.relative_to(ROOT)}")

# Ensure this overlay cannot overwrite frozen/validated project layers.
for forbidden_dir in ("frozen_parent", "s2_4_shadow", "tools", "python_tests"):
    if (ROOT / forbidden_dir).exists():
        errors.append(f"overlay unexpectedly contains protected directory: {forbidden_dir}")

# Static source contract copied from the full audit, excluding parent audits.
decision_files = [
    C / "mission" / "exploration_request_S2_4.m",
    C / "mission" / "plan_active_exploration_segment_S2_4.m",
    C / "execution" / "build_execution_grid_S2_4.m",
    C / "execution" / "project_uncertainty_2d_S2_4.m",
    C / "execution" / "validate_exploration_request_S2_4.m",
    C / "execution" / "summarize_viewpoint_decision_S2_4.m",
]
forbidden = [
    r"scenario\.truth", r"truthWorld", r"truthStatic", r"truthDynamic",
    r"geometric_controller", r"motor_mixer", r"thrust_N\s*=", r"moment_Nm\s*=",
    r"velocityCommand", r"attitudeCommand", r"bodyRateCommand",
    r"validationGeometry",
]
for path in decision_files:
    if not path.is_file():
        continue
    text = path.read_text(encoding="utf-8")
    for pattern in forbidden:
        if re.search(pattern, text, flags=re.I):
            errors.append(f"forbidden decision dependency {pattern!r}: {path.relative_to(ROOT)}")

if (C / "mission" / "exploration_request_S2_4.m").is_file():
    text = (C / "mission" / "exploration_request_S2_4.m").read_text(encoding="utf-8")
    for token in ("S2_4_EXPLORATION_REQUEST_V1", "commandIssued',false", "MOVE_TO_VIEWPOINT"):
        if token not in text:
            errors.append(f"request schema token missing: {token}")

if (C / "mission" / "mission_lifecycle_manager_S2_4.m").is_file():
    text = (C / "mission" / "mission_lifecycle_manager_S2_4.m").read_text(encoding="utf-8")
    for token in (
        "plan_active_exploration_segment_S2_4", "validate_exploration_request_S2_4",
        "explorationRequestCount", "explorationExecutedCount", "unsafeViewpointExecutionCount",
        "uncertaintySidecar", "explorationRequestHistory", "explorationDecisionHistory",
        "targetRelevantSelectionPass",
    ):
        if token not in text:
            errors.append(f"mission coupling token missing: {token}")

if (C / "mission" / "run_S2_4_coupled.m").is_file():
    text = (C / "mission" / "run_S2_4_coupled.m").read_text(encoding="utf-8")
    if "fullfile(projectRoot,'results','S2_4_coupled'" not in text:
        errors.append("runner does not use project-level S2_4_coupled results folder")
    if "fullfile(parentRoot,'results'" in text:
        errors.append("runner writes inside frozen parent")

if (C / "scenarios" / "scenario_S2_4.m").is_file():
    text = (C / "scenarios" / "scenario_S2_4.m").read_text(encoding="utf-8")
    for token in ("make_literal_competing_corridors_S2_4", "expectedMinCompetingFrontiers", "expectedMinDistinctIrrelevantFrontiers"):
        if token not in text:
            errors.append(f"literal competing-corridor scenario token missing: {token}")

if (C / "mission" / "init_S2_4_E_config.m").is_file():
    text = (C / "mission" / "init_S2_4_E_config.m").read_text(encoding="utf-8")
    version_match = re.search(r"cfg\.version\s*=\s*'([^']+)'", text)
    if not version_match:
        errors.append("competing-corridor behavior version declaration missing")
    else:
        behavior_version = version_match.group(1)
        if not re.fullmatch(
            r"v\d+\.\d+\.\d+-(?:literal|adversarial)-competing-corridors-candidate",
            behavior_version,
        ):
            errors.append(f"competing-corridor behavior version invalid: {behavior_version}")

geom = C / "scenarios" / "literal_competing_corridors_geometry.json"
if geom.is_file():
    data = json.loads(geom.read_text(encoding="utf-8"))
    for token in ("fork_occluder", "east_branch_divider", "decoy_probe_xy_m"):
        if token not in geom.read_text(encoding="utf-8"):
            errors.append(f"literal corridor geometry token missing: {token}")
    if data.get("schema") != "S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1":
        errors.append("literal geometry schema mismatch")

# Basic MATLAB source hygiene.
for path in required:
    if not path.is_file() or path.suffix != ".m":
        continue
    text = path.read_text(encoding="utf-8")
    if "TODO" in text or "FIXME" in text:
        errors.append(f"unfinished marker: {path.relative_to(ROOT)}")
    first = next((line.strip() for line in text.splitlines() if line.strip()), "")
    if not first.startswith("function"):
        errors.append(f"MATLAB file does not begin with function: {path.relative_to(ROOT)}")
    if text.count("(") != text.count(")"):
        errors.append(f"parenthesis imbalance: {path.relative_to(ROOT)}")
    if text.count("[") != text.count("]"):
        errors.append(f"bracket imbalance: {path.relative_to(ROOT)}")

# Python syntax.
for p in C.rglob("*.py"):
    try:
        py_compile.compile(str(p), doraise=True)
    except Exception as exc:  # pragma: no cover
        errors.append(f"Python compile failed: {p.relative_to(ROOT)}: {exc}")

if errors:
    print("S2.4-E CUMULATIVE OVERLAY PORTABLE AUDIT: FAIL")
    for e in errors:
        print(" -", e)
    raise SystemExit(1)

print(f"Required cumulative files checked: {len(required)}")
print("Protected project directories absent from overlay: PASS")
print("Decision truth/command isolation (source-level): PASS")
print("Mission-request schema (source-level): PASS")
print("Literal competing-corridor source contract: PASS")
print("Python syntax: PASS")

cmd = [sys.executable, str(C / "validation" / "audit_S2_4_E_literal_corridor_geometry.py")]
print("$", " ".join(cmd))
p = subprocess.run(cmd, cwd=ROOT)
if p.returncode:
    raise SystemExit(p.returncode)
print("S2.4-E CUMULATIVE OVERLAY PORTABLE AUDIT: PASS")
print("NOTE: full parent/A-D/recorded-replay gates require installation into the user's complete project root.")
