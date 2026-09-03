#!/usr/bin/env python3
"""Static audit that G v1.0.3 removes only the nominal exploration expectation
from critical fault-run qualification, while preserving every other component
of the mission manager's composite pass.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
V=(ROOT/'coupled'/'validation'/'validate_S2_4_G_all.m').read_text()
M=(ROOT/'coupled'/'mission'/'mission_lifecycle_manager_S2_4.m').read_text()

# Mission-manager full pass ingredients are fixed by source.
pass_line='pass=missionLifecyclePass&&trajectoryGate&&controllerGate&&estimatorGate&&continuityPass&&uncertaintyPass&&staticGate&&mappingPass&&explorationPass&&~stateTimeoutTriggered;'
assert pass_line in M
assert 'if executionSafetyEnabled,pass=pass&&executionSafetyPass;end' in M

# Critical fault-run mission completion is literal lifecycle completion and does
# not re-import the complete nominal summary.pass.
crit=V[V.index('function q=qualifyCriticalRun'):V.index('function tf=criticalClosedLoopIntegrity')]
assert "strcmp(s.finalState,'COMPLETE')" in crit
assert '&&s.pass' not in crit.replace('nominalScenario=s.pass','')
assert 'nominalScenario=s.pass' in crit
assert 'nominalExploration=s.explorationPass' in crit

core=V[V.index('function tf=criticalClosedLoopIntegrity'):V.index('function r=emptyBaselineRecord')]
required=(
    's.missionOutcomePass','s.trajectoryGate','s.controllerGate','s.estimatorGate',
    's.continuityPass','s.uncertaintyPass','s.staticGate','s.mappingCompositePass',
    's.executionSafetyPass','~s.stateTimeoutTriggered')
for token in required: assert token in core, token
assert 's.explorationPass' not in core

# No-fault baselines still demand the full nominal pass, so E's original
# exploration-execution contract has not been weakened globally.
base=V[V.index('function q=qualifyNoFault'):V.index('function q=qualifyCriticalRun')]
assert '&&s.pass' in base

# Critical q.pass must require both true mission completion and the preserved
# closed-loop composite.
assert '&&mission&&closedLoop' in crit

print('S2.4-G mission-manager full composite source contract located: PASS')
print('S2.4-G no-fault baselines still require full nominal summary.pass: PASS')
print('S2.4-G critical mission is literal COMPLETE/goal/non-timeout outcome: PASS')
print('S2.4-G critical closed-loop gate preserves every non-exploration composite gate: PASS')
print('S2.4-G critical gate excludes only nominal explorationPass: PASS')
print('S2.4-G v1.0.3 QUALIFICATION SEMANTICS STATIC AUDIT: PASS')
