#!/usr/bin/env python3
"""Static gate for S2.4-E mission-request coupling candidate."""
from __future__ import annotations
import hashlib
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
COUPLED = ROOT / "coupled"
PARENT = ROOT / "frozen_parent" / "S2_3_online_mapping_v1_0_0_validated"

required = [
    COUPLED / "mission" / "init_S2_4_E_config.m",
    COUPLED / "mission" / "exploration_request_S2_4.m",
    COUPLED / "mission" / "plan_active_exploration_segment_S2_4.m",
    COUPLED / "mission" / "mission_lifecycle_manager_S2_4.m",
    COUPLED / "mission" / "run_S2_4_coupled.m",
    COUPLED / "execution" / "build_execution_grid_S2_4.m",
    COUPLED / "execution" / "project_uncertainty_2d_S2_4.m",
    COUPLED / "execution" / "validate_exploration_request_S2_4.m",
    COUPLED / "execution" / "summarize_viewpoint_decision_S2_4.m",
    COUPLED / "scenarios" / "scenario_S2_4.m",
    COUPLED / "scenarios" / "make_literal_competing_corridors_S2_4.m",
    COUPLED / "scenarios" / "literal_competing_corridors_geometry.json",
    COUPLED / "validation" / "validate_S2_4_E_milestone_1.m",
    COUPLED / "validation" / "validate_S2_4_E_milestone_2.m",
    COUPLED / "validation" / "validate_S2_4_E_competing_corridors_multiseed.m",
    COUPLED / "validation" / "test_S2_4_E_competing_decision_contract.m",
    COUPLED / "validation" / "test_S2_4_E_literal_corridor_geometry_contract.m",
    COUPLED / "validation" / "audit_S2_4_E_literal_corridor_geometry.py",
    COUPLED / "validation" / "test_S2_4_E_request_contracts.m",
    COUPLED / "validation" / "validate_S2_4_E_all.m",
]

errors: list[str] = []
for path in required:
    if not path.is_file():
        errors.append(f"missing: {path.relative_to(ROOT)}")

# Frozen parent must remain byte-identical.
for script in ("audit_parent_immutability.py", "audit_final_parent_manifest.py"):
    p = subprocess.run([sys.executable, str(ROOT / "tools" / script)], cwd=ROOT,
                       text=True, capture_output=True)
    print(p.stdout, end="")
    if p.returncode:
        errors.append(f"parent audit failed: {script}")

# Decision/request modules may not access scenario truth or emit low-level commands.
decision_files = [
    COUPLED / "mission" / "exploration_request_S2_4.m",
    COUPLED / "mission" / "plan_active_exploration_segment_S2_4.m",
    COUPLED / "execution" / "build_execution_grid_S2_4.m",
    COUPLED / "execution" / "project_uncertainty_2d_S2_4.m",
    COUPLED / "execution" / "validate_exploration_request_S2_4.m",
    COUPLED / "execution" / "summarize_viewpoint_decision_S2_4.m",
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

# The request schema must explicitly remain a mission request, not a command.
request_text = (COUPLED / "mission" / "exploration_request_S2_4.m").read_text(encoding="utf-8")
for token in ("S2_4_EXPLORATION_REQUEST_V1", "commandIssued',false", "MOVE_TO_VIEWPOINT"):
    if token not in request_text:
        errors.append(f"request schema token missing: {token}")

manager_text = (COUPLED / "mission" / "mission_lifecycle_manager_S2_4.m").read_text(encoding="utf-8")
for token in (
    "plan_active_exploration_segment_S2_4",
    "validate_exploration_request_S2_4",
    "explorationRequestCount",
    "explorationExecutedCount",
    "unsafeViewpointExecutionCount",
    "uncertaintySidecar",
    "explorationRequestHistory",
    "explorationDecisionHistory",
    "targetRelevantSelectionPass",
):
    if token not in manager_text:
        errors.append(f"mission coupling token missing: {token}")

# New outputs must be outside frozen_parent.
runner_text = (COUPLED / "mission" / "run_S2_4_coupled.m").read_text(encoding="utf-8")
if "fullfile(projectRoot,'results','S2_4_coupled'" not in runner_text:
    errors.append("runner does not use project-level S2_4_coupled results folder")
if "fullfile(parentRoot,'results'" in runner_text:
    errors.append("runner writes inside frozen parent")

# Literal Scenario-2 must be a physical fork overlay, not the old tall-screen alias.
scenario_text = (COUPLED / "scenarios" / "scenario_S2_4.m").read_text(encoding="utf-8")
for token in (
    "make_literal_competing_corridors_S2_4",
    "expectedMinCompetingFrontiers",
    "expectedMinDistinctIrrelevantFrontiers",
):
    if token not in scenario_text:
        errors.append(f"literal competing-corridor scenario token missing: {token}")
config_text = (COUPLED / "mission" / "init_S2_4_E_config.m").read_text(encoding="utf-8")
version_match = re.search(r"cfg\.version\s*=\s*'([^']+)'", config_text)
if not version_match:
    errors.append("S2.4-E behavior version declaration missing")
else:
    behavior_version = version_match.group(1)
    if not re.fullmatch(
        r"v\d+\.\d+\.\d+-(?:literal|adversarial)-competing-corridors-candidate",
        behavior_version,
    ):
        errors.append(
            f"S2.4-E competing-corridor behavior version invalid: {behavior_version}"
        )
geometry_path = COUPLED / "scenarios" / "literal_competing_corridors_geometry.json"
if geometry_path.is_file():
    geometry_text = geometry_path.read_text(encoding="utf-8")
    for token in ("fork_occluder", "east_branch_divider", "decoy_probe_xy_m"):
        if token not in geometry_text:
            errors.append(f"literal corridor geometry token missing: {token}")

# MATLAB arguments blocks: required arguments must precede optional defaults.
for path in required:
    if not path.is_file() or path.suffix != ".m":
        continue
    lines = path.read_text(encoding="utf-8").splitlines()
    in_arguments = False
    optional_seen = False
    for lineno, line in enumerate(lines, start=1):
        stripped = line.strip()
        if stripped == "arguments":
            in_arguments = True
            optional_seen = False
            continue
        if not in_arguments:
            continue
        if stripped == "end":
            in_arguments = False
            continue
        if not stripped or stripped.startswith("%"):
            continue
        has_default = "=" in stripped
        if has_default:
            optional_seen = True
        elif optional_seen:
            errors.append(
                f"required argument follows optional argument: "
                f"{path.relative_to(ROOT)}:{lineno}: {stripped}"
            )

# Lightweight source hygiene: one primary function per file and no placeholders.
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

print(f"Coupled MATLAB files checked: {sum(p.is_file() for p in required)}")
if errors:
    print("S2.4-E STATIC GATE: FAIL")
    for e in errors:
        print(" -", e)
    raise SystemExit(1)
print("Decision truth/command isolation: PASS")
print("Mission-request schema: PASS")
print("Frozen-parent output isolation: PASS")
print("S2.4-E STATIC GATE: PASS")
