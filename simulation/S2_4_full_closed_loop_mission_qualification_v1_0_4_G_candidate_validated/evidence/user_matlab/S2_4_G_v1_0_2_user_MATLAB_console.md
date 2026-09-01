
\============================================================
 S2.4 A-D COMPLETE OFFLINE/SHADOW VALIDATION
 Session : 20260829\_150251
 Root    : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate
\============================================================


[GATE 0] PACKAGE STRUCTURE
Package structure: PASS


[GATE 1] PYTHON ENVIRONMENT

Python 3.9.6

NumPy 2.0.2
SciPy 1.13.1
h5py 3.12.1

Python environment: PASS

[GATE 2] S2.4 STATIC/OFFLINE PACKAGE GATE


$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_parent\_immutability.py
Frozen S2.3 files checked: 208
Missing: 0 | changed: 0 | extra: 0
PARENT BYTE IDENTITY: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_final\_parent\_manifest.py
Final closure manifest paths: 203; mismatches: 0
Mismatches: none
All source files match final closure manifest: 1
FINAL PARENT MANIFEST INTERPRETATION: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_truth\_isolation\_s2\_4.py
S2.4 truth/command isolation: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/matlab\_source\_sanity\_s2\_4.py
MATLAB source files checked: 9
MATLAB SOURCE SANITY: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 python\_tests/s2\_4\_ad\_contract\_backtest.py
01 scenario\_1               PASS 
02 scenario\_2               PASS 
03 scenario\_3               PASS 
04 scenario\_4               PASS 
05 scenario\_5               PASS 
06 scenario\_6               PASS 
07 scenario\_7               PASS 

08 scenario\_8               PASS 
09 scenario\_9               PASS 
10 scenario\_10              PASS 
11 scenario\_11              PASS 
12 scenario\_12              PASS 
13 scenario\_13              PASS 
14 scenario\_14              PASS 
15 scenario\_15              PASS 

S2.4 A-D PACKAGE STATIC/OFFLINE GATE: PASS

S2.4 static/offline package gate: PASS

[GATE 3] S2.3 MATLAB PATH ISOLATION


Resolved S2.3 functions:
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/S2\_3\_online\_mapping\_v1\_0\_0\_validated/run\_S2\_3\_online\_mapping.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/S2\_3\_online\_mapping\_v1\_0\_0\_validated/validate\_S2\_3\_release\_all.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/S2\_3\_online\_mapping\_v1\_0\_0\_validated/replay\_perception\_log\_S2\_3.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/S2\_3\_online\_mapping\_v1\_0\_0\_validated/update\_probabilistic\_map\_S2\_3.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/S2\_3\_online\_mapping\_v1\_0\_0\_validated/mission\_lifecycle\_manager\_S2\_3.m
S2.3 MATLAB path isolation: PASS

[GATE 4] COMPLETE S2.3 RELEASE REGRESSION: SKIPPED

[GATE 5] FRESH S2.3 NOMINAL COUPLED TRACE

\============================================================
 STAGE S2.3 v1.0.0-candidate PERCEPTION-DRIVEN ONLINE MAPPING
 seed=0 | scenario=UNKNOWN\_ROOM\_NOMINAL | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/results/S2\_3\_online\_mapping/v1\_0\_0\_candidate/unknown\_room\_nominal/seed\_000
\============================================================

Goal/unreachable/failsafe : 1 / 0 / 0
Arm/takeoff/RTL/land      : 1 / 1 / 1 / 1
Map version/packets       : 1077 / 1342 accepted / 0 rejected / 2684 idle
Replay records captured   : 1342
Extensions done/planned    : 1 / 1 | scans/route repairs/all replans 1 / 0 / 0
Map false-free/recall     : 0.00175 / 0.990
Map observed fraction     : 0.880 | promotions 1125
Unknown commitments       : 0 | truth isolation 1
Collision / geofence      : 0 / 0
Tracking / estimator max  : 0.019 / 0.034 m
Reference XY v/a/j        : 0.315 / 0.425 / 1.129
Executed XY v/a/j         : 0.334 / 0.526 / 2.087
Mapping/event/mission     : 1 / 1 / 1
Core T/C/E/S/MC           : 1 / 1 / 1 / 1 / 1
Trajectories/grid fallback: 3 / 0
Map gates M/E/R/S/U/T     : 1 / 1 / 1 / 1 / 1 / 1
Final state               : COMPLETE
RESULT                    : PASS


Fresh nominal trace:
/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/results/S2\_3\_online\_mapping/v1\_0\_0\_candidate/unknown\_room\_nominal/seed\_000/S2\_3\_v1\_0\_0\_candidate\_trial\_data.mat
Fresh S2.3 nominal coupled trace: PASS

[GATE 6] EXACT S2.3 PERCEPTION-MAPPER REPLAY


\============================================================
 S2.3 EXACT PERCEPTION-MAPPER REPLAY
 Records / accepted / rejected : 1342 / 1342 / 0
 Exact arrays / core counters  : 1 / 1
 Idle counters replay/coupled  : 0 / 2684 (informational)
 Original false-free / recall  : 0.00174985 / 0.990054
 Replay false-free / recall    : 0.00174985 / 0.990054
 Metrics exact match           : 1
 REPLAY RESULT                 : PASS
\============================================================

Exact inherited S2.3 mapper replay: PASS

[GATE 7] PYTHON RECORDED-TRACE S2.4 REPLAY


$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_parent\_immutability.py
Frozen S2.3 files checked: 208
Missing: 0 | changed: 0 | extra: 0
PARENT BYTE IDENTITY: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_final\_parent\_manifest.py
Final closure manifest paths: 203; mismatches: 0
Mismatches: none
All source files match final closure manifest: 1
FINAL PARENT MANIFEST INTERPRETATION: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/audit\_truth\_isolation\_s2\_4.py
S2.4 truth/command isolation: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 tools/matlab\_source\_sanity\_s2\_4.py
MATLAB source files checked: 9
MATLAB SOURCE SANITY: PASS

$ /Library/Developer/CommandLineTools/usr/bin/python3 python\_tests/s2\_4\_ad\_contract\_backtest.py
01 scenario\_1               PASS 
02 scenario\_2               PASS 
03 scenario\_3               PASS 
04 scenario\_4               PASS 
05 scenario\_5               PASS 
06 scenario\_6               PASS 

07 scenario\_7               PASS 
08 scenario\_8               PASS 
09 scenario\_9               PASS 
10 scenario\_10              PASS 
11 scenario\_11              PASS 
12 scenario\_12              PASS 
13 scenario\_13              PASS 
14 scenario\_14              PASS 
15 scenario\_15              PASS 

$ /Library/Developer/CommandLineTools/usr/bin/python3 python\_tests/s2\_4\_recorded\_shadow\_replay.py /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/frozen\_parent/results/S2\_3\_online\_mapping/v1\_0\_0\_candidate/unknown\_room\_nominal/seed\_000/S2\_3\_v1\_0\_0\_candidate\_trial\_data.mat --output-dir evidence
S2.4 A-D recorded S2.3 shadow replay
scenario: UNKNOWN\_ROOM\_NOMINAL
snapshots: 81
incremental frontier == full: 1
deterministic repeat: 1
unsafe accepted candidates: 0
tracks created: 19
replay digest: 60216caa557e1679c5bb3b657fd240e574ae33a48a8b8579c781fa5cfd899901
uncertainty digest: 89e610ed70e9e7a58e869eab637791c142a179eb4f59b0659186b0aaf52fbd6e
RESULT: PASS

S2.4 A-D PACKAGE STATIC/OFFLINE GATE: PASS

Python recorded-trace S2.4 replay: PASS

[GATE 8] S2.4 MATLAB PATH ISOLATION


Resolved S2.4 functions:
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/s2\_4\_shadow/validate\_S2\_4\_AD.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/s2\_4\_shadow/run\_S2\_4\_AD\_shadow\_replay.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/s2\_4\_shadow/extract\_frontiers\_incremental\_S2\_4.m
  /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/s2\_4\_shadow/generate\_safe\_viewpoints\_S2\_4.m
S2.4 MATLAB path isolation: PASS

[GATE 9] MATLAB S2.4 A-D TWO-RUN SHADOW GATE


\============================================================
 S2.3 EXACT PERCEPTION-MAPPER REPLAY
 Records / accepted / rejected : 1342 / 1342 / 0
 Exact arrays / core counters  : 1 / 1
 Idle counters replay/coupled  : 0 / 2684 (informational)
 Original false-free / recall  : 0.00174985 / 0.990054
 Replay false-free / recall    : 0.00174985 / 0.990054
 Metrics exact match           : 1
 REPLAY RESULT                 : PASS
\============================================================


S2.4 A-D SHADOW REPLAY: PASS


\============================================================
 S2.3 EXACT PERCEPTION-MAPPER REPLAY
 Records / accepted / rejected : 1342 / 1342 / 0
 Exact arrays / core counters  : 1 / 1
 Idle counters replay/coupled  : 0 / 2684 (informational)
 Original false-free / recall  : 0.00174985 / 0.990054
 Replay false-free / recall    : 0.00174985 / 0.990054
 Metrics exact match           : 1
 REPLAY RESULT                 : PASS
\============================================================


S2.4 A-D SHADOW REPLAY: PASS

S2.4 MATLAB gate values:
                 parentExactMapperReplay: 1
                       mapperArraysExact: 1
          uncertaintyReplayDeterministic: 1
    frontierViewpointReplayDeterministic: 1
                       repeatCountsExact: 1
                 frontierReplayCompleted: 1
                  acceptedCandidateCount: 146
                unsafeAcceptedCandidates: 0
                          truthIsolation: 1
                        commandIsolation: 1
                                    pass: 1

MATLAB S2.4 A-D two-run shadow gate: PASS

[GATE 10] RESULT-EVIDENCE FILES

Replay summary:
  Snapshots processed       : 81
  Frontier tracks           : 40
  Accepted candidates       : 146
  Unsafe accepted candidates: 0

  Truth accesses            : 0
  Commands issued           : 0
Result-evidence files: PASS

[GATE 11] POST-RUN PARENT BYTE IDENTITY

Frozen S2.3 files checked: 208
Missing: 0 | changed: 0 | extra: 0
PARENT BYTE IDENTITY: PASS
Final closure manifest paths: 203; mismatches: 0
Mismatches: none
All source files match final closure manifest: 1
FINAL PARENT MANIFEST INTERPRETATION: PASS

Post-run frozen-parent byte identity: PASS


\============================================================
 S2.4 A-D VALIDATION COMPLETE
\============================================================
 S2.4 static/offline matrix       : PASS
 S2.3 inherited regression        : PASS
 S2.3 exact mapper replay         : PASS
 Python recorded replay           : PASS
 MATLAB uncertainty determinism   : PASS
 MATLAB frontier determinism      : PASS

 Unsafe accepted viewpoints       : 0
 Truth-isolation violations       : 0
 Commands issued                  : 0
 Parent byte identity after run   : PASS
 FINAL S2.4 A-D SHADOW GATE       : PASS
\============================================================

Results folder:
/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_AD\_validation\_20260829\_150251

Validation ZIP:
/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_AD\_validation\_20260829\_150251.zip

MATLAB console log:
/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_AD\_validation\_20260829\_150251/MATLAB\_complete\_console.txt


S2.4-E MATLAB PATH SETUP: PASS
 run\_S2\_4\_coupled             : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/coupled/mission/run\_S2\_4\_coupled.m
 exploration\_request\_S2\_4    : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/coupled/mission/exploration\_request\_S2\_4.m
 validate\_S2\_4\_E\_all          : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/coupled/validation/validate\_S2\_4\_E\_all.m


\============================================================
 S2.4-G PHASE 2 — F RUNTIME DELTA + E/F COUPLED QUALIFICATION
\============================================================

S2.4-G F-v1.1.2 baseline delta audit: checked=271 missing=0 changed=1
CHANGED coupled/mission/mission\_lifecycle\_manager\_S2\_4.m 4d2ff497e602dcb8fbaa33c5d49898d4eca77242da3e4080dfcc6df1a8a27310 -> 8d7a137f660eb00a3460b4d562b42c31c895e48b26f9f7ec50d5f36446998550
S2.4-G REVIEWED F RUNTIME DELTA: PASS (exactly mission\_lifecycle\_manager\_S2\_4.m)

01 valid\_known\_free\_request                         PASS
02 blocked\_route\_rejected                           PASS
03 unknown\_viewpoint\_rejected                       PASS
04 local\_hold\_support\_rejected                      PASS
05 expired\_request\_rejected                         PASS
06 unrelated\_map\_version\_change\_revalidated         PASS
S2.4-E REQUEST CONTRACTS: 6/6 PASS

S2.4-E CONTROLLED ADVERSARIAL POLICY CONTRACT: PASS
target IG / utility: 5.000 / 0.320
decoy  IG / utility: 12.000 / 0.470
                    twoFrontiers: 1
                     competition: 1
            targetSafetyFeasible: 1
             decoySafetyFeasible: 1
                 decoyPolicyOnly: 1
         decoyHasMoreInformation: 1
        decoyHasHigherRawUtility: 1

                     targetTier1: 1
                      decoyTier3: 1
                  targetSelected: 1
           noIrrelevantSelection: 1
                   distinctDecoy: 1
    mostInformativeDecoyRecorded: 1
      rawUtilityDecoyBeatsTarget: 1
                tierPriorityWins: 1


S2.4-E LITERAL CORRIDOR GEOMETRY CONTRACT: PASS
Safe fork width upper/lower   : 0.646 / 0.896 m
Safe branch width target/decoy: 0.996 / 2.196 m
Obstacle schema source          : inherited\:hidden\_obstacle\_replan
Physical obstacle overlay:
    2.4500    2.1000    0.4000    2.0500
    2.8500    3.4000    3.0000    0.4000


\============================================================
 S2.4-E v0.3.6-adversarial-competing-corridors-candidate COUPLED ACTIVE EXPLORATION
 seed=0 | scenario=ACTIVE\_GOAL\_REQUIRES\_SCAN | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_goal\_requires\_scan/seed\_000
\============================================================

Goal/unreachable/failsafe       : 1 / 0 / 0
Exploration request/selected    : 1 / 1
Viewpoints executed             : 1
Competing decisions/frontiers   : 1 / 3
Target/irrelevant selections    : 1 / 0
First selected tier/relevance   : 1 / 4.893
Best decoy information gain     : 28.068
Unsafe viewpoint execution steps: 0
Unknown commitments             : 0
Collision / geofence            : 0 / 0
Exploration / mapping / mission : 1 / 1 / 1
Final state                     : COMPLETE
RESULT                          : PASS


S2.4-E MILESTONE 1 GATE: PASS
                     missionPass: 1
                requestGenerated: 1
                 requestAccepted: 1
               viewpointExecuted: 1
                     goalReached: 1
                   rtlAndLanding: 1
                   zeroCollision: 1
                    zeroGeofence: 1
           zeroUnknownCommitment: 1
    zeroUnsafeViewpointExecution: 1
                  truthIsolation: 1
                            pass: 1


S2.4-E CONTROLLED ADVERSARIAL POLICY CONTRACT: PASS
target IG / utility: 5.000 / 0.320
decoy  IG / utility: 12.000 / 0.470
                    twoFrontiers: 1
                     competition: 1
            targetSafetyFeasible: 1
             decoySafetyFeasible: 1
                 decoyPolicyOnly: 1
         decoyHasMoreInformation: 1
        decoyHasHigherRawUtility: 1
                     targetTier1: 1
                      decoyTier3: 1
                  targetSelected: 1
           noIrrelevantSelection: 1
                   distinctDecoy: 1
    mostInformativeDecoyRecorded: 1
      rawUtilityDecoyBeatsTarget: 1
                tierPriorityWins: 1


S2.4-E LITERAL CORRIDOR GEOMETRY CONTRACT: PASS
Safe fork width upper/lower   : 0.646 / 0.896 m
Safe branch width target/decoy: 0.996 / 2.196 m
Obstacle schema source          : inherited\:hidden\_obstacle\_replan
Physical obstacle overlay:
    2.4500    2.1000    0.4000    2.0500
    2.8500    3.4000    3.0000    0.4000


\============================================================
 S2.4-E v0.3.6-adversarial-competing-corridors-candidate COUPLED ACTIVE EXPLORATION
 seed=0 | scenario=ACTIVE\_COMPETING\_CORRIDORS | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_competing\_corridors/seed\_000
\============================================================

Goal/unreachable/failsafe       : 1 / 0 / 0
Exploration request/selected    : 1 / 1
Viewpoints executed             : 1
Competing decisions/frontiers   : 1 / 3
Target/irrelevant selections    : 1 / 0
First selected tier/relevance   : 1 / 1.964
Best decoy information gain     : 9.964
Unsafe viewpoint execution steps: 0
Unknown commitments             : 0
Collision / geofence            : 0 / 0
Exploration / mapping / mission : 1 / 1 / 1
Final state                     : COMPLETE
RESULT                          : PASS


S2.4-E MILESTONE 2 — LAYERED LITERAL COMPETING CORRIDORS: PASS
Controlled adversarial policy contract: 1
Selected frontier / target relevance / tier: 1 / 1.964386 / 1
Selected physical information gain          : 17.793070
Clean decoy frontier / candidate / IG       : 2 / 100 / 9.964386
Physical decoy/selected IG ratio (observed): 0.560015
Clean decoy rejection reasons               : IRRELEVANT\_EXPLORATION
Physical decoy more informative (observation): 0
               adversarialPolicyContract: 1

                 literalGeometryContract: 1
                  literalTruthWorldMatch: 1
                             missionPass: 1
                        requestGenerated: 1
                         requestAccepted: 1
                       viewpointExecuted: 1
               competingDecisionObserved: 1
                     atLeastTwoFrontiers: 1
              irrelevantCandidatePresent: 1
       distinctIrrelevantFrontierPresent: 1
               cleanFeasibleDecoyPresent: 1
                  targetRelevantSelected: 1
                   noIrrelevantSelection: 1
                    tierPriorityRecorded: 1
                             goalReached: 1
                           rtlAndLanding: 1
                           zeroCollision: 1
                            zeroGeofence: 1
                   zeroUnknownCommitment: 1
            zeroUnsafeViewpointExecution: 1
                          truthIsolation: 1
    physicalDecoyMoreInformativeObserved: 0
            decoyStrictlyMoreInformative: 0
                                    pass: 1


S2.4-E COMBINED MATLAB GATE — MILESTONES 1+2: PASS
S2.4-F deterministic revalidation contracts: PASS
F15 predictive moving-obstacle contract: NOT APPLICABLE (live predictor not connected)


\============================================================
 S2.4-E v0.3.6-adversarial-competing-corridors-candidate COUPLED ACTIVE EXPLORATION
 seed=0 | scenario=ACTIVE\_GOAL\_REQUIRES\_SCAN | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_goal\_requires\_scan/seed\_000
\============================================================

Goal/unreachable/failsafe       : 1 / 0 / 0
Exploration request/selected    : 1 / 1
Viewpoints executed             : 1
Competing decisions/frontiers   : 1 / 3
Target/irrelevant selections    : 1 / 0
First selected tier/relevance   : 1 / 4.893
Best decoy information gain     : 28.068
Unsafe viewpoint execution steps: 0
Unknown commitments             : 0
Collision / geofence            : 0 / 0
Exploration / mapping / mission : 1 / 1 / 1
Final state                     : COMPLETE
RESULT                          : PASS

S2.4-F no-fault E reference parity: PASS

F2: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F3: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F4: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F5: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F6: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=0 authGen=1

F7: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=0 authGen=1

F8: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F9: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F10: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F11: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=0 authGen=1

F13: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=1 authGen=2

F14: PASS | injected=1 detected=1 response=1 stale=0 collision=0 unknown=0 invalidations=3 authGen=3

S2.4-F EXECUTION-TIME SAFETY MATLAB GATE: PASS

\============================================================
 S2.4-G PHASE 2B — NO-FAULT SEED BASELINES (5 RUNS)
\============================================================

BASE seed=0 : PASS | mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0(out=0 rtl=0) reauth=0

BASE seed=1 : PASS | mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0(out=0 rtl=0) reauth=0

BASE seed=2 : PASS | mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0(out=0 rtl=0) reauth=0

BASE seed=3 : PASS | mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0(out=0 rtl=0) reauth=0

BASE seed=4 : PASS | mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0(out=0 rtl=0) reauth=0

\============================================================
 S2.4-G PHASE 3 — TARGETED CRITICAL FAULT ROBUSTNESS (75 RUNS)
\============================================================

01/75 F2 EARLY seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

02/75 F2 EARLY seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

03/75 F2 EARLY seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=1 reauth=0

04/75 F2 EARLY seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

05/75 F2 EARLY seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

06/75 F2 MID   seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

07/75 F2 MID   seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

08/75 F2 MID   seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

09/75 F2 MID   seed=3 : FAIL | inj=1 det=1 resp=1 mission=0 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0
      reason=VALID invalidations=1 retreatRefresh=1 perceptionRevoke=0 freshPlans=1 reauth=0 reject=1 timeout=0 goalUnreachable=0 final=COMPLETE tGoal=57.90 tComplete=108.04 unkOut=0 unkRTL=0 guardOut=0 guardRTL=0 mapTruth=0 uncertaintyTruth=0

10/75 F2 MID   seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

11/75 F2 LATE  seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

12/75 F2 LATE  seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

13/75 F2 LATE  seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

14/75 F2 LATE  seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

15/75 F2 LATE  seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

16/75 F3 EARLY seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

17/75 F3 EARLY seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

18/75 F3 EARLY seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=1 reauth=0

19/75 F3 EARLY seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

20/75 F3 EARLY seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

21/75 F3 MID   seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

22/75 F3 MID   seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

23/75 F3 MID   seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

24/75 F3 MID   seed=3 : FAIL | inj=1 det=1 resp=1 mission=0 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0
      reason=VALID invalidations=1 retreatRefresh=1 perceptionRevoke=0 freshPlans=1 reauth=0 reject=1 timeout=0 goalUnreachable=0 final=COMPLETE tGoal=57.90 tComplete=108.04 unkOut=0 unkRTL=0 guardOut=0 guardRTL=0 mapTruth=0 uncertaintyTruth=0

25/75 F3 MID   seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

26/75 F3 LATE  seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

27/75 F3 LATE  seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

28/75 F3 LATE  seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

29/75 F3 LATE  seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

30/75 F3 LATE  seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

31/75 F6 EARLY seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

32/75 F6 EARLY seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

33/75 F6 EARLY seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

34/75 F6 EARLY seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

35/75 F6 EARLY seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

36/75 F6 MID   seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

37/75 F6 MID   seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

38/75 F6 MID   seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

39/75 F6 MID   seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

40/75 F6 MID   seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

41/75 F6 LATE  seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

42/75 F6 LATE  seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

43/75 F6 LATE  seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

44/75 F6 LATE  seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

45/75 F6 LATE  seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

46/75 F9 EARLY seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

47/75 F9 EARLY seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

48/75 F9 EARLY seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=1 reauth=0

49/75 F9 EARLY seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

50/75 F9 EARLY seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

51/75 F9 MID   seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

52/75 F9 MID   seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

53/75 F9 MID   seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

54/75 F9 MID   seed=3 : FAIL | inj=1 det=1 resp=1 mission=0 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0
      reason=VALID invalidations=1 retreatRefresh=0 perceptionRevoke=1 freshPlans=1 reauth=0 reject=1 timeout=0 goalUnreachable=0 final=COMPLETE tGoal=57.90 tComplete=108.04 unkOut=0 unkRTL=0 guardOut=0 guardRTL=0 mapTruth=0 uncertaintyTruth=0

55/75 F9 MID   seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

56/75 F9 LATE  seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

57/75 F9 LATE  seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

58/75 F9 LATE  seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

59/75 F9 LATE  seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

60/75 F9 LATE  seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

61/75 F10 EARLY seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

62/75 F10 EARLY seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

63/75 F10 EARLY seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=1 reauth=0

64/75 F10 EARLY seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

65/75 F10 EARLY seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

66/75 F10 MID   seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

67/75 F10 MID   seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

68/75 F10 MID   seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

69/75 F10 MID   seed=3 : FAIL | inj=1 det=1 resp=1 mission=0 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0
      reason=VALID invalidations=1 retreatRefresh=0 perceptionRevoke=0 freshPlans=1 reauth=0 reject=1 timeout=0 goalUnreachable=0 final=COMPLETE tGoal=57.90 tComplete=108.04 unkOut=0 unkRTL=0 guardOut=0 guardRTL=0 mapTruth=0 uncertaintyTruth=0

70/75 F10 MID   seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=0

71/75 F10 LATE  seed=0 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

72/75 F10 LATE  seed=1 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

73/75 F10 LATE  seed=2 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

74/75 F10 LATE  seed=3 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

75/75 F10 LATE  seed=4 : PASS | inj=1 det=1 resp=1 mission=1 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 refGuard=0 reauth=1

\============================================================
 S2.4-G PHASE 4 — POST-RUN RUNTIME/FROZEN-PARENT INTEGRITY
\============================================================

Frozen S2.3 files checked: 208
Missing: 0 | changed: 0 | extra: 0
PARENT BYTE IDENTITY: PASS
Final closure manifest paths: 203; mismatches: 0
Mismatches: none
All source files match final closure manifest: 1
FINAL PARENT MANIFEST INTERPRETATION: PASS

S2.4-G F-v1.1.2 baseline delta audit: checked=271 missing=0 changed=1
CHANGED coupled/mission/mission\_lifecycle\_manager\_S2\_4.m 4d2ff497e602dcb8fbaa33c5d49898d4eca77242da3e4080dfcc6df1a8a27310 -> 8d7a137f660eb00a3460b4d562b42c31c895e48b26f9f7ec50d5f36446998550
S2.4-G REVIEWED F RUNTIME DELTA: PASS (exactly mission\_lifecycle\_manager\_S2\_4.m)


\============================================================
 S2.4-G FULL CLOSED-LOOP MISSION QUALIFICATION: FAIL
 A-D shadow qualification       : PASS (completed before this function)
 F runtime reviewed delta       : PASS
 E + F coupled qualification    : PASS
 No-fault seed baselines        : 5/5 PASS
 Critical timing/seed matrix    : 71/75 PASS
 Mission completion             : FAIL
 Hard safety                    : PASS

 Actual truth access isolation  : PASS
 Frozen-parent integrity        : PASS
 Results                        : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_4\_full\_closed\_loop\_mission\_qualification\_v1\_0\_2\_G\_candidate/results/S2\_4\_G\_qualification\_20260829\_171816
\============================================================

Error using [**assert**](matlab\:matlab.lang.internal.introspective.errorDocCallback\('assert'\))
S2.4-G full closed-loop qualification failed.

Error in [**run\_validate\_S2\_4\_G\_all**](<matlab\:matlab.lang.internal.introspective.errorDocCallback('run_validate_S2_4_G_all', '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_full_closed_loop_mission_qualification_v1_0_2_G_candidate/run_validate_S2_4_G_all.m', 24)>) ([line 24](<matlab: opentoline('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_full_closed_loop_mission_qualification_v1_0_2_G_candidate/run_validate_S2_4_G_all.m',24,0)>))
assert(gate.pass,'S2\_4\:GValidationFailed','S2.4-G full closed-loop qualification failed.');
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


\>> 