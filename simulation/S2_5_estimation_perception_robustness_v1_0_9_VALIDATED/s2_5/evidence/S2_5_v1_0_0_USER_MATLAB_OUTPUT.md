\>> gate = run\_validate\_S2\_5\_all();

S2.5 MATLAB PATH SETUP: PASS
 run\_S2\_5\_coupled              : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/s2\_5/mission/run\_S2\_5\_coupled.m
 mission\_lifecycle\_manager\_S2\_5: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/s2\_5/mission/mission\_lifecycle\_manager\_S2\_5.m
 mission\_lifecycle\_manager\_S2\_4: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/coupled/mission/mission\_lifecycle\_manager\_S2\_4.m

S2.5 MATLAB PATH SETUP: PASS
 run\_S2\_5\_coupled              : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/s2\_5/mission/run\_S2\_5\_coupled.m
 mission\_lifecycle\_manager\_S2\_5: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/s2\_5/mission/mission\_lifecycle\_manager\_S2\_5.m
 mission\_lifecycle\_manager\_S2\_4: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/coupled/mission/mission\_lifecycle\_manager\_S2\_4.m

\============================================================
 S2.5 PHASE 0 — VALIDATED S2.4-G PARENT + STATIC CONTRACTS
\============================================================

S2.4-G validated parent files checked: 353
Missing: 0 | changed: 0
S2.4-G VALIDATED PARENT BYTE IDENTITY: PASS

S2.4-G validated parent files checked: 353
Missing: 0 | changed: 0
S2.4-G VALIDATED PARENT BYTE IDENTITY: PASS
required\_files                                       PASS
parent\_byte\_identity                                 PASS
inherits\_validated\_F\_config                          PASS
no\_estimator\_threshold\_relaxation                    PASS
no\_map\_safety\_threshold\_relaxation                   PASS
execution\_validation\_fault\_disabled                  PASS
manager\_delta\_only\_packet\_hooks\_and\_diagnostics      PASS
scenario\_S25\_NAV\_VIO\_DROPOUT                         PASS
scenario\_S25\_NAV\_LIDAR\_AID\_DROPOUT                   PASS
scenario\_S25\_NAV\_VIO\_OUTLIER\_BURST                   PASS
scenario\_S25\_NAV\_LIDAR\_OUTLIER\_BURST                 PASS

scenario\_S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE        PASS
scenario\_S25\_NAV\_HIGH\_MEASUREMENT\_NOISE              PASS
scenario\_S25\_NAV\_XY\_AID\_LOSS\_FAILSAFE                PASS
scenario\_S25\_PERCEPTION\_LIDAR\_DROPOUT                PASS
scenario\_S25\_PERCEPTION\_DEPTH\_DROPOUT                PASS
scenario\_S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT           PASS
scenario\_S25\_PERCEPTION\_STALE\_PACKET\_BURST           PASS
scenario\_S25\_PERCEPTION\_RANGE\_SPIKE                  PASS
scenario\_S25\_PERCEPTION\_PROLONGED\_DUAL\_DROPOUT       PASS
scenario\_S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT    PASS
nav\_outlier\_hooks                                    PASS
perception\_fault\_hooks                               PASS
S2.5 STATIC / ISOLATION AUDIT: PASS

N3-VIO-NIS   PASS  lower-bound NIS 62.59 > gate 27.877
N4-LID-NIS   PASS  lower-bound NIS 76.24 > gate 16.266
N5-IMU       PASS  acc 0.466>0.220, gyro 2.166>1.200 deg/s, 4 samples
P3-HOLD      PASS  1.20 s > hold 0.55 and < failsafe 4.00
P4-STALE     PASS  lag 0.45 s > packet-age gate 0.30 s
P6-FAILSAFE  PASS  7.0 s > perception failsafe 4.00 s
P5-STATIC    PASS  persistent static occupancy is not cleared by free-ray evidence
FAIL-CLOSED  PASS  planner projection retains explicit known-free/unknown state
N6-NOISE     PASS  expected VIO/LiDAR NIS 20.25/6.75 below gates
WINDOWS      PASS  events VIO=16 navLiDAR=4 rawLiDAR=1 depth=1
S2.5 SOURCE-FAITHFUL FAULT / GATE BACKTEST: PASS

\============================================================
 S2.5 PHASE 1 — INHERITED S2.4-F COUPLED REGRESSION
\============================================================

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
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_goal\_requires\_scan/seed\_000
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
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_competing\_corridors/seed\_000
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
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_4\_coupled/v0\_3\_6\_adversarial\_competing\_corridors\_candidate/active\_goal\_requires\_scan/seed\_000
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
 S2.5 PHASE 2 — NO-FAULT BASELINES (5 SEEDS)
\============================================================


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_BASELINE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_baseline/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

BASE seed=0 : PASS | E=1 pos=0.0158 att=1.1293 laneSw=0 mapReject=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_BASELINE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_baseline/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1755 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

BASE seed=1 : PASS | E=1 pos=0.0259 att=1.0632 laneSw=0 mapReject=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_BASELINE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_baseline/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1653 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

BASE seed=2 : PASS | E=1 pos=0.0296 att=1.1844 laneSw=0 mapReject=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_BASELINE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_baseline/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

BASE seed=3 : PASS | E=1 pos=0.0317 att=0.9989 laneSw=0 mapReject=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_BASELINE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_baseline/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

BASE seed=4 : PASS | E=1 pos=0.0313 att=1.1636 laneSw=0 mapReject=0

\============================================================
 S2.5 PHASE 3 — RECOVERABLE ESTIMATION/PERCEPTION MATRIX (60 RUNS)
\============================================================


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_VIO\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0194 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 151 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

01/60 NAV\_VIO\_DROPOUT                 seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=151 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_VIO\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0276 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1755 / 0 / 0
S2.5 nav/perception fault apps  : 151 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

02/60 NAV\_VIO\_DROPOUT                 seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=151 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_VIO\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1653 / 0 / 0
S2.5 nav/perception fault apps  : 151 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

03/60 NAV\_VIO\_DROPOUT                 seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=151 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_VIO\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 151 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

04/60 NAV\_VIO\_DROPOUT                 seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=151 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_VIO\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0343 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 151 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

05/60 NAV\_VIO\_DROPOUT                 seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=151 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_LIDAR\_AID\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_aid\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 31 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

06/60 NAV\_LIDAR\_DROPOUT               seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=31 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_LIDAR\_AID\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_aid\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1755 / 0 / 0
S2.5 nav/perception fault apps  : 31 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

07/60 NAV\_LIDAR\_DROPOUT               seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=31 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_LIDAR\_AID\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_aid\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1653 / 0 / 0
S2.5 nav/perception fault apps  : 31 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

08/60 NAV\_LIDAR\_DROPOUT               seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=31 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_LIDAR\_AID\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_aid\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 31 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

09/60 NAV\_LIDAR\_DROPOUT               seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=31 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_LIDAR\_AID\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_aid\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 31 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

10/60 NAV\_LIDAR\_DROPOUT               seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=31 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_VIO\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_outlier\_burst/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 16 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

11/60 NAV\_VIO\_OUTLIER                 seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=16 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_VIO\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_outlier\_burst/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1840 / 0 / 0
S2.5 nav/perception fault apps  : 16 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

12/60 NAV\_VIO\_OUTLIER                 seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=16 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_VIO\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_outlier\_burst/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1653 / 0 / 0
S2.5 nav/perception fault apps  : 16 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

13/60 NAV\_VIO\_OUTLIER                 seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=16 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_VIO\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_outlier\_burst/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 16 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

14/60 NAV\_VIO\_OUTLIER                 seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=16 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_VIO\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_vio\_outlier\_burst/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 16 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

15/60 NAV\_VIO\_OUTLIER                 seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=16 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_LIDAR\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_outlier\_burst/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 4 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

16/60 NAV\_LIDAR\_OUTLIER               seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=4 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_LIDAR\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_outlier\_burst/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1755 / 0 / 0
S2.5 nav/perception fault apps  : 4 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

17/60 NAV\_LIDAR\_OUTLIER               seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=4 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_LIDAR\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_outlier\_burst/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1653 / 0 / 0
S2.5 nav/perception fault apps  : 4 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

18/60 NAV\_LIDAR\_OUTLIER               seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=4 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_LIDAR\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_outlier\_burst/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 4 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

19/60 NAV\_LIDAR\_OUTLIER               seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=4 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_LIDAR\_OUTLIER\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_lidar\_outlier\_burst/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 4 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

20/60 NAV\_LIDAR\_OUTLIER               seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=4 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_primary\_imu\_fault\_vio\_outage/seed\_000
\============================================================

21/60 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=0 : PASS | E=1 laneSw=1 hold=0 reject=0 navF=251 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_primary\_imu\_fault\_vio\_outage/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 0 / 0
Estimator pos/att/gate          : 0.0304 m / 1.0632 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 638 / 0 / 0
S2.5 nav/perception fault apps  : 251 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

22/60 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=1 : FAIL | E=1 laneSw=1 hold=0 reject=0 navF=251 perF=0 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0304 att=1.0632 unknown=0 coll=0 geo=0 extra=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_primary\_imu\_fault\_vio\_outage/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0396 m / 1.1844 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 251 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

23/60 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=2 : PASS | E=1 laneSw=1 hold=0 reject=0 navF=251 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_primary\_imu\_fault\_vio\_outage/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0357 m / 0.9989 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1707 / 0 / 0
S2.5 nav/perception fault apps  : 251 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

24/60 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=3 : PASS | E=1 laneSw=1 hold=0 reject=0 navF=251 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_primary\_imu\_fault\_vio\_outage/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0362 m / 1.1636 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1685 / 0 / 0
S2.5 nav/perception fault apps  : 251 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

25/60 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=4 : PASS | E=1 laneSw=1 hold=0 reject=0 navF=251 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_HIGH\_MEASUREMENT\_NOISE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_high\_measurement\_noise/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0274 m / 1.6938 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1673 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

26/60 NAV\_HIGH\_NOISE                  seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_HIGH\_MEASUREMENT\_NOISE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_high\_measurement\_noise/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0389 m / 1.5949 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1642 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

27/60 NAV\_HIGH\_NOISE                  seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_HIGH\_MEASUREMENT\_NOISE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_high\_measurement\_noise/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0444 m / 1.7762 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1634 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

28/60 NAV\_HIGH\_NOISE                  seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_NAV\_HIGH\_MEASUREMENT\_NOISE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_high\_measurement\_noise/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0475 m / 1.4980 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1704 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

29/60 NAV\_HIGH\_NOISE                  seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_NAV\_HIGH\_MEASUREMENT\_NOISE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_high\_measurement\_noise/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0470 m / 1.7459 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1683 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

30/60 NAV\_HIGH\_NOISE                  seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=0 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_LIDAR\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_lidar\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1661 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 67
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

31/60 PERCEPTION\_LIDAR\_DROPOUT        seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=67 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_LIDAR\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_lidar\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1702 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 67
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

32/60 PERCEPTION\_LIDAR\_DROPOUT        seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=67 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_LIDAR\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_lidar\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1600 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 67
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

33/60 PERCEPTION\_LIDAR\_DROPOUT        seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=67 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_PERCEPTION\_LIDAR\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_lidar\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1648 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 67
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

34/60 PERCEPTION\_LIDAR\_DROPOUT        seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=67 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_PERCEPTION\_LIDAR\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_lidar\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1692 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 67
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

35/60 PERCEPTION\_LIDAR\_DROPOUT        seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=67 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_DEPTH\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_depth\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1647 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 81
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

36/60 PERCEPTION\_DEPTH\_DROPOUT        seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=81 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_DEPTH\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_depth\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1688 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 81
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

37/60 PERCEPTION\_DEPTH\_DROPOUT        seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=81 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_DEPTH\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_depth\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1586 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 81
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

38/60 PERCEPTION\_DEPTH\_DROPOUT        seed=2 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=81 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_PERCEPTION\_DEPTH\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_depth\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1634 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 81
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

39/60 PERCEPTION\_DEPTH\_DROPOUT        seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=81 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_PERCEPTION\_DEPTH\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_depth\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1678 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 81
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

40/60 PERCEPTION\_DEPTH\_DROPOUT        seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=81 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_brief\_dual\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1905 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

41/60 PERCEPTION\_DUAL\_BRIEF           seed=0 : PASS | E=1 laneSw=0 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_brief\_dual\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 660 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

42/60 PERCEPTION\_DUAL\_BRIEF           seed=1 : FAIL | E=1 laneSw=0 hold=1 reject=0 navF=0 perF=21 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_brief\_dual\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1782 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

43/60 PERCEPTION\_DUAL\_BRIEF           seed=2 : PASS | E=1 laneSw=0 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_brief\_dual\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1816 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

44/60 PERCEPTION\_DUAL\_BRIEF           seed=3 : PASS | E=1 laneSw=0 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_brief\_dual\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1885 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

45/60 PERCEPTION\_DUAL\_BRIEF           seed=4 : PASS | E=1 laneSw=0 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_STALE\_PACKET\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_stale\_packet\_burst/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1905 / 21 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

46/60 PERCEPTION\_STALE\_BURST          seed=0 : PASS | E=1 laneSw=0 hold=1 reject=21 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_STALE\_PACKET\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_stale\_packet\_burst/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 660 / 21 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

47/60 PERCEPTION\_STALE\_BURST          seed=1 : FAIL | E=1 laneSw=0 hold=1 reject=21 navF=0 perF=21 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_STALE\_PACKET\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_stale\_packet\_burst/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1782 / 21 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

48/60 PERCEPTION\_STALE\_BURST          seed=2 : PASS | E=1 laneSw=0 hold=1 reject=21 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_PERCEPTION\_STALE\_PACKET\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_stale\_packet\_burst/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1816 / 21 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

49/60 PERCEPTION\_STALE\_BURST          seed=3 : PASS | E=1 laneSw=0 hold=1 reject=21 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_PERCEPTION\_STALE\_PACKET\_BURST | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_stale\_packet\_burst/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1885 / 21 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

50/60 PERCEPTION\_STALE\_BURST          seed=4 : PASS | E=1 laneSw=0 hold=1 reject=21 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_RANGE\_SPIKE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_range\_spike/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1714 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 1
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

51/60 PERCEPTION\_RANGE\_SPIKE          seed=0 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=1 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_RANGE\_SPIKE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_range\_spike/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1755 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 1
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

52/60 PERCEPTION\_RANGE\_SPIKE          seed=1 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=1 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_RANGE\_SPIKE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_range\_spike/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1185 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 1
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

53/60 PERCEPTION\_RANGE\_SPIKE          seed=2 : FAIL | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=1 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0296 att=1.1844 unknown=0 coll=0 geo=0 extra=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_PERCEPTION\_RANGE\_SPIKE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_range\_spike/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1701 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 1
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

54/60 PERCEPTION\_RANGE\_SPIKE          seed=3 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=1 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_PERCEPTION\_RANGE\_SPIKE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_range\_spike/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 1745 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 1
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

55/60 PERCEPTION\_RANGE\_SPIKE          seed=4 : PASS | E=1 laneSw=0 hold=0 reject=0 navF=0 perF=1 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_coupled\_imu\_fault\_perception\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1909 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

56/60 COUPLED\_IMU\_PERCEPTION          seed=0 : PASS | E=1 laneSw=1 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_coupled\_imu\_fault\_perception\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 0 / 0
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 2514 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

57/60 COUPLED\_IMU\_PERCEPTION          seed=1 : FAIL | E=1 laneSw=1 hold=1 reject=0 navF=0 perF=21 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_coupled\_imu\_fault\_perception\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1907 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

58/60 COUPLED\_IMU\_PERCEPTION          seed=2 : PASS | E=1 laneSw=1 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=3 | scenario=S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_coupled\_imu\_fault\_perception\_dropout/seed\_003
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0317 m / 0.9989 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1785 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

59/60 COUPLED\_IMU\_PERCEPTION          seed=3 : PASS | E=1 laneSw=1 hold=1 reject=0 navF=0 perF=21 safety=1


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=4 | scenario=S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_coupled\_imu\_fault\_perception\_dropout/seed\_004
\============================================================

Mission/goal/failsafe/emergency : 1 / 1 / 0 / 0
Estimator pos/att/gate          : 0.0313 m / 1.1636 deg / 1
Lane switches / active final    : 1 / 2
Map accepted/rejected/hold      : 1814 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 21
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / PASS
Final state                     : COMPLETE

60/60 COUPLED\_IMU\_PERCEPTION          seed=4 : PASS | E=1 laneSw=1 hold=1 reject=0 navF=0 perF=21 safety=1

\============================================================
 S2.5 PHASE 4 — FAIL-SAFE ROBUSTNESS (6 RUNS)
\============================================================


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_NAV\_XY\_AID\_LOSS\_FAILSAFE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_xy\_aid\_loss\_failsafe/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.0574 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 408 / 0 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

01/6 NAV\_XY\_LOSS                     seed=0 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0574 att=1.1293 geo=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_NAV\_XY\_AID\_LOSS\_FAILSAFE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_xy\_aid\_loss\_failsafe/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.1462 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 411 / 7 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

02/6 NAV\_XY\_LOSS                     seed=1 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.1462 att=1.0632 geo=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_NAV\_XY\_AID\_LOSS\_FAILSAFE | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_nav\_xy\_aid\_loss\_failsafe/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.2173 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 411 / 1 / 0
S2.5 nav/perception fault apps  : 0 / 0
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

03/6 NAV\_XY\_LOSS                     seed=2 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.2173 att=1.1844 geo=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=0 | scenario=S25\_PERCEPTION\_PROLONGED\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_prolonged\_dual\_dropout/seed\_000
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.0158 m / 1.1293 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 432 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 118
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

04/6 PERCEPTION\_DUAL\_PROLONGED       seed=0 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0158 att=1.1293 geo=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=1 | scenario=S25\_PERCEPTION\_PROLONGED\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_prolonged\_dual\_dropout/seed\_001
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.0259 m / 1.0632 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 372 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 118
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

05/6 PERCEPTION\_DUAL\_PROLONGED       seed=1 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 geo=0


\============================================================
 S2.5 v1.0.0-estimation-perception-robustness-candidate ESTIMATION + PERCEPTION ROBUSTNESS
 seed=2 | scenario=S25\_PERCEPTION\_PROLONGED\_DUAL\_DROPOUT | plots=0 | animation=0
 Results: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_estimation\_perception/v1\_0\_0\_estimation\_perception\_robustness\_candidate/s25\_perception\_prolonged\_dual\_dropout/seed\_002
\============================================================

Mission/goal/failsafe/emergency : 1 / 0 / 1 / 1
Estimator pos/att/gate          : 0.0296 m / 1.1844 deg / 1
Lane switches / active final    : 0 / 1
Map accepted/rejected/hold      : 372 / 0 / 1
S2.5 nav/perception fault apps  : 0 / 118
Unknown/collision/geofence      : 0 / 0 / 0
Truth isolation / S2.5 case     : 1 / FAIL
Final state                     : COMPLETE

06/6 PERCEPTION\_DUAL\_PROLONGED       seed=2 : FAIL | failsafe=1 emergency=1 complete=1 unknown=0 coll=0 truth=1
      final=COMPLETE goal=0 timeout=0 gates[M=1 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0296 att=1.1844 geo=0

\============================================================
 S2.5 PHASE 5 — POST-RUN PARENT IMMUTABILITY
\============================================================

S2.4-G validated parent files checked: 353
Missing: 0 | changed: 0
S2.4-G VALIDATED PARENT BYTE IDENTITY: PASS


\============================================================
 S2.5 ESTIMATION + PERCEPTION ROBUSTNESS: FAIL
 Validated S2.4-G parent        : PASS
 Inherited S2.4-F regression    : PASS
 No-fault baselines             : 5/5 PASS
 Recoverable fault matrix       : 55/60 PASS
 Fail-safe fault matrix         : 0/6 PASS
 Results                        : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_0\_candidate/results/S2\_5\_qualification/20260830\_234954
\============================================================

Error using [**assert**](matlab\:matlab.lang.internal.introspective.errorDocCallback\('assert'\))
S2.5 estimation/perception robustness gate failed.

Error in [**run\_validate\_S2\_5\_all**](<matlab\:matlab.lang.internal.introspective.errorDocCallback('run_validate_S2_5_all', '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_0_candidate/run_validate_S2_5_all.m', 6)>) ([line 6](<matlab: opentoline('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_0_candidate/run_validate_S2_5_all.m',6,0)>))
assert(gate.pass,'S2\_5\:ValidationFailed','S2.5 estimation/perception robustness gate failed.');
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^