# S2.4 — Uncertainty-Aware Target-Directed Active Exploration

**Release candidate:** `v1.0.0-RC3 clean`  
**Status:** final clean-build MATLAB parity check pending  
**Parent:** `S2_3_online_mapping_v1_0_0_validated`

This is the complete S2.4 milestone: A–D uncertainty/frontier/viewpoint development plus the final S2.4-E coupled mission integration.

## Retained feature scope
- read-only uncertainty reasoning over the authoritative S2.3 probabilistic map;
- persistent frontier extraction and tracking;
- safe target-directed viewpoint generation;
- known-free route, stopping-support and retreat validation;
- Tier 1 target > Tier 2 safe progress > Tier 3 unrelated exploration;
- mission-level exploration requests and latest-map request revalidation;
- viewpoint execution, scan/map update and target replanning;
- strict truth isolation;
- goal completion, RTL and landing;
- controlled adversarial policy contract and physical multiseed validation.

## Clean release layout
- `config/` — S2.4 configuration
- `s2_4_shadow/` — A–D uncertainty/frontier/viewpoint implementation
- `coupled/` — final coupled runtime and V6 layered validation
- `python_tests/`, `tools/` — deterministic/static validation
- `recorded_inputs/` — retained S2.3 replay input
- `frozen_parent/` — exact S2.3 source + only the legacy trace required by the selector regression
- `docs/`, `evidence/` — design and validation evidence
- `poster_figures/` — presentation assets
- `results/` — intentionally empty until clean-build verification

Development-only material is excluded: Python-first search harness, old milestone ZIPs, backups, caches, obsolete patch audits and historical full result trees.

## Final verification
1. Terminal: `python3 coupled/validation/run_all_checks_S2_4_E.py`
2. MATLAB: `run_validate_S2_4_AD_all`
3. fresh MATLAB session
4. MATLAB: `run_validate_S2_4_release_candidate`

Expected parity target is in `evidence/final/PRE_CLEANUP_VALIDATION_REFERENCE.md`.

If all checks reproduce PASS, make no executable changes; rename/package as `S2_4_uncertainty_active_exploration_v1_0_0_validated`.
