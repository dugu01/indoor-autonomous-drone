S2.5 MATLAB PATH SETUP: PASS
 run\_S2\_5\_coupled              : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/s2\_5/mission/run\_S2\_5\_coupled.m
 mission\_lifecycle\_manager\_S2\_5: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/s2\_5/mission/mission\_lifecycle\_manager\_S2\_5.m
 mission\_lifecycle\_manager\_S2\_4: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/coupled/mission/mission\_lifecycle\_manager\_S2\_4.m

S2.5 MATLAB PATH SETUP: PASS
 run\_S2\_5\_coupled              : /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/s2\_5/mission/run\_S2\_5\_coupled.m
 mission\_lifecycle\_manager\_S2\_5: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/s2\_5/mission/mission\_lifecycle\_manager\_S2\_5.m
 mission\_lifecycle\_manager\_S2\_4: /Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2\_5\_estimation\_perception\_robustness\_v1\_0\_2\_parallel\_candidate\_r2/coupled/mission/mission\_lifecycle\_manager\_S2\_4.m

\============================================================
 S2.5 PHASE 0 — VALIDATED S2.4-G PARENT + STATIC CONTRACTS
\============================================================

S2.4-G validated parent files checked: 353
Missing: 0 | changed: 0
S2.4-G VALIDATED PARENT BYTE IDENTITY: PASS

S2.4-G validated parent files checked: 353
Missing: 0 | changed: 0
S2.4-G VALIDATED PARENT BYTE IDENTITY: PASS
required\_files                                                 PASS
parent\_byte\_identity                                           PASS
inherits\_validated\_F\_config                                    PASS
candidate\_version\_v1\_0\_2                                       PASS
no\_estimator\_threshold\_relaxation                              PASS
no\_map\_safety\_threshold\_relaxation                             PASS
execution\_validation\_fault\_disabled                            PASS

v101\_unchanged\_scenarios\_scenario\_S2\_5\_m                       PASS
v101\_unchanged\_sensors\_simulate\_sensor\_packet\_S2\_5\_m           PASS
v101\_unchanged\_perception\_simulate\_perception\_packet\_S2\_5\_m    PASS
serial\_runner\_reference\_exact                                  PASS
runner\_reviewed\_io\_delta                                       PASS
config\_delta\_version\_only                                      PASS
manager\_changed\_from\_v101                                      PASS
episode\_counter                                                PASS
episode\_bound                                                  PASS
fresh\_request\_resets\_episode                                   PASS
same\_request\_reauth\_does\_not\_reset                             PASS
conservative\_fallback\_counter                                  PASS
fallback\_reason                                                PASS
bound\_fallback\_reason                                          PASS
frozen\_s23\_planner\_reused                                      PASS
three\_scans\_semantics                                          PASS
episode\_invalidation\_limit\_unchanged                           PASS
mission\_extension\_limit\_unchanged                              PASS
manager\_no\_threshold\_or\_gain\_tuning                            PASS
historical\_failure\_preflight                                   PASS
failsafe\_helper\_found                                          PASS
failsafe\_no\_nominal\_mapping\_completeness                       PASS
failsafe\_keeps\_hard\_safety                                     PASS
scenario\_S25\_NAV\_VIO\_DROPOUT                                   PASS
scenario\_S25\_NAV\_LIDAR\_AID\_DROPOUT                             PASS
scenario\_S25\_NAV\_VIO\_OUTLIER\_BURST                             PASS
scenario\_S25\_NAV\_LIDAR\_OUTLIER\_BURST                           PASS
scenario\_S25\_NAV\_PRIMARY\_IMU\_FAULT\_VIO\_OUTAGE                  PASS
scenario\_S25\_NAV\_HIGH\_MEASUREMENT\_NOISE                        PASS
scenario\_S25\_NAV\_XY\_AID\_LOSS\_FAILSAFE                          PASS
scenario\_S25\_PERCEPTION\_LIDAR\_DROPOUT                          PASS
scenario\_S25\_PERCEPTION\_DEPTH\_DROPOUT                          PASS
scenario\_S25\_PERCEPTION\_BRIEF\_DUAL\_DROPOUT                     PASS
scenario\_S25\_PERCEPTION\_STALE\_PACKET\_BURST                     PASS
scenario\_S25\_PERCEPTION\_RANGE\_SPIKE                            PASS
scenario\_S25\_PERCEPTION\_PROLONGED\_DUAL\_DROPOUT                 PASS
scenario\_S25\_COUPLED\_IMU\_FAULT\_PERCEPTION\_DROPOUT              PASS
nav\_outlier\_hooks                                              PASS
perception\_fault\_hooks                                         PASS
S2.5 v1.0.2 STATIC / ISOLATION AUDIT: PASS

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

trace\_NAV\_IMU\_FAULT\_VIO\_OUTAGE                                     PASS 
trace\_PERCEPTION\_DUAL\_BRIEF                                        PASS 
trace\_PERCEPTION\_STALE\_BURST                                       PASS 
trace\_PERCEPTION\_RANGE\_SPIKE                                       PASS 
trace\_COUPLED\_IMU\_PERCEPTION                                       PASS 
trace\_direct\_goal\_never\_available                                  PASS 
trace\_no\_safe\_active\_viewpoint\_repeats                             PASS 
trace\_authority\_bound\_cases                                        PASS 
trace\_global\_count\_mixed\_requests                                  PASS 
episode\_source\_authorityEpisodeRequestId\_uint64\_0\_authorityE       PASS 
episode\_source\_authorityEpisodeInvalidationCount\_authorityEp       PASS 
episode\_source\_authorityEpisodeRequestId\_active\_request\_id\_p       PASS 
episode\_source\_authorityEpisodeInvalidationCount\_0\_                PASS 
episode\_source\_pendingExplorationTerminal\_authorityEpisodeIn       PASS 
episode\_source\_field\_or\_local\_executionSafetyCfg\_maxAuthorit       PASS 
episode\_increment\_three\_revocation\_seams                           PASS 
reauthorization\_does\_not\_reset\_episode                             PASS 
old\_global\_false\_terminal                                          PASS 
new\_episode\_not\_terminal\_for\_req25                                 PASS 
same\_request\_three\_invalidations\_terminal                          PASS 
fallback\_on\_no\_safe\_active\_viewpoint                               PASS 
bound\_branch\_found                                                 PASS 
bound\_uses\_frozen\_s23\_planner                                      PASS 
bound\_does\_not\_issue\_active\_request                                PASS 
bound\_allows\_frontier\_not\_only\_direct                              PASS 
s23\_frontier\_requires\_known\_free                                   PASS 
s23\_frontier\_requires\_goal\_progress                                PASS 
s23\_frontier\_requires\_astar                                        PASS 
s23\_plan\_uses\_strict\_trajectory                                    PASS 
s23\_plan\_uses\_known\_free\_stop                                      PASS 
bound\_clears\_dead\_request                                          PASS 
conservative\_hop\_reopens\_fresh\_episode                             PASS 
mission\_level\_extension\_bound\_unchanged                            PASS 
three\_configured\_scans\_still\_allowed                               PASS 
failsafe\_helper\_found                                              PASS 
failsafe\_nominal\_map\_completeness\_not\_required                     PASS 
failsafe\_keeps\_mapFalseFreeRate                                    PASS 
failsafe\_keeps\_unknownCommitmentCount\_0                            PASS 
failsafe\_keeps\_truthIsolationPass                                  PASS 

failsafe\_keeps\_missionOutcomePass                                  PASS 
failsafe\_keeps\_estimatorGate                                       PASS 
failsafe\_keeps\_executionSafetyPass                                 PASS 
failsafe\_keeps\_failsafeTriggered                                   PASS 
failsafe\_keeps\_emergencyLanding                                    PASS 
failsafe\_keeps\_missionComplete                                     PASS 
preflight\_\_nav\_imu\_fault\_vio\_outage\_                               PASS 
preflight\_\_perception\_dual\_brief\_                                  PASS 
preflight\_\_perception\_stale\_burst\_                                 PASS 
preflight\_\_perception\_range\_spike\_                                 PASS 
preflight\_\_coupled\_imu\_perception\_                                 PASS 
preflight\_histSeeds\_1\_1\_1\_2\_1\_                                     PASS 
preflight\_historical\_cached\_run                                    PASS 
preflight\_assert\_all\_histPass\_                                     PASS 
version\_v1\_0\_2                                                     PASS 
estimator\_thresholds\_untouched                                     PASS 
map\_thresholds\_untouched                                           PASS 
execution\_fault\_injection\_disabled                                 PASS 
negative\_no\_route                                                  PASS 
negative\_bad\_trajectory                                            PASS 
negative\_bad\_stop                                                  PASS 
positive\_known\_free\_frontier                                       PASS 
S2.5 v1.0.2 RECOVERY / FAIL-SAFE BACKTEST: PASS

v102\_mission\_manager\_byte\_identity                                   PASS 9dbfa7390ea39a8b030d6fabe7a5c4e29b770fe1d9564b8e25f3d38bfe53342a
runtime\_file\_present\_mission\_init\_S2\_5\_config\_m                      PASS 
runtime\_file\_present\_scenarios\_scenario\_S2\_5\_m                       PASS 
runtime\_file\_present\_sensors\_simulate\_sensor\_packet\_S2\_5\_m           PASS 
runtime\_file\_present\_perception\_simulate\_perception\_packet\_S2\_5\_m    PASS 
runner\_core\_rng\_seed\_twister\_                                        PASS 
runner\_core\_cfg\_init\_S2\_5\_config\_cfg\_seed\_seed\_                      PASS 
runner\_core\_scenario\_scenario\_S2\_5\_scenarioName\_                     PASS 
runner\_core\_\_log\_summary\_maps\_mission\_lifecycle\_manager\_S2\_5\_cfg\_sc  PASS 
runner\_core\_summary\_s25CasePass\_evaluate\_s25\_case\_summary\_scenario\_  PASS 
runner\_optional\_save\_defaults\_true                                   PASS 
runner\_optional\_verbose\_defaults\_true                                PASS 
qualification\_runs\_full\_logic\_compact                                PASS 
qualification\_returns\_summary\_only                                   PASS 
acceptance\_semantics\_\_pass\_extra\_historical\_case\_pass                PASS parallel=407 serial=407
acceptance\_semantics\_\_pass\_extra\_recoverable\_case\_pass               PASS parallel=529 serial=529
acceptance\_semantics\_\_pass\_mapSafety\_core\_evidence\_failsafe\_case\_pass PASS parallel=719 serial=719
parallel\_hist\_preflight                                              PASS 
parallel\_baselines                                                   PASS 
parallel\_recoverable\_jobs                                            PASS 
parallel\_failsafe\_jobs                                               PASS 
inherited\_F\_remains\_serial                                           PASS 
toolbox\_required                                                     PASS 
workers\_bounded\_to\_8                                                 PASS 
process\_pool\_preferred                                               PASS 
worker\_override\_supported                                            PASS 
python\_spawn\_bootstrap\_guard                                         PASS 
python\_spawn\_context\_explicit                                        PASS 
hist\_exact\_5                                                         PASS 
recoverable\_exact\_60                                                 PASS 
historical\_subset\_of\_recoverable                                     PASS 
noncached\_recoverable\_exact\_55                                       PASS 
qualification\_exact\_71\_unique\_missions                               PASS 71
parallel\_execution\_exact\_same\_71                                     PASS 
rng\_reseed\_per\_case                                                  PASS 
result\_dir\_scenario\_seed\_isolated                                    PASS 
parallel\_case\_keys\_unique                                            PASS 
python\_spawn\_parallel\_order\_independent                              PASS 
recoverable\_matrix\_indexing\_order\_independent                        PASS 
historical\_failfast\_before\_inherited\_and\_matrix                      PASS 
historical\_results\_cached                                            PASS 
normal\_runner\_still\_can\_save\_full\_artifacts                          PASS 
qualification\_disables\_full\_artifact\_save                            PASS 
gate\_report\_still\_persisted                                          PASS 

S2.5 PARALLEL HARNESS PYTHON BACKTEST: PASS
Unique coupled missions: 71 | historical cached: 5 | new recoverable worker jobs: 55

Starting parallel pool (parpool) using the 'Processes' profile ...

Connected to parallel pool with 8 workers.

S2.5 PARALLEL POOL: PASS | workers=8 | profile=Processes

\============================================================
 S2.5 PHASE 0C — PARALLEL HISTORICAL RECOVERY PREFLIGHT (5 CACHED RUNS)
\============================================================

1/5 NAV\_IMU\_FAULT\_VIO\_OUTAGE        seed=1 : FAIL | goal=0 E=1 laneSw=1 hold=0 inv=1 epInv=1 epBound=0 fb=0 scans=91 extra=1 safety=1
      final=SCAN\_HOLD goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0307 att=1.0632 unknown=0 coll=0 geo=0 extra=1 inv=1 suppress=0 scans=91
      recovery[episodeInv=1 episodeBound=0 conservativeFallback=0]
      unreachableReason=PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET
2/5 PERCEPTION\_DUAL\_BRIEF           seed=1 : FAIL | goal=0 E=1 laneSw=0 hold=1 inv=5 epInv=1 epBound=0 fb=0 scans=31 extra=1 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1 inv=5 suppress=0 scans=31
      recovery[episodeInv=1 episodeBound=0 conservativeFallback=0]
      unreachableReason=PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET
3/5 PERCEPTION\_STALE\_BURST          seed=1 : FAIL | goal=0 E=1 laneSw=0 hold=1 inv=5 epInv=1 epBound=0 fb=0 scans=31 extra=1 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1 inv=5 suppress=0 scans=31
      recovery[episodeInv=1 episodeBound=0 conservativeFallback=0]
      unreachableReason=PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET
4/5 PERCEPTION\_RANGE\_SPIKE          seed=2 : FAIL | goal=0 E=1 laneSw=0 hold=0 inv=0 epInv=0 epBound=0 fb=0 scans=4 extra=1 safety=1
      final=COMPLETE goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0296 att=1.1844 unknown=0 coll=0 geo=0 extra=1 inv=0 suppress=0 scans=4
      recovery[episodeInv=0 episodeBound=0 conservativeFallback=0]
      unreachableReason=PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET
5/5 COUPLED\_IMU\_PERCEPTION          seed=1 : FAIL | goal=0 E=1 laneSw=1 hold=1 inv=2 epInv=1 epBound=0 fb=0 scans=90 extra=1 safety=1
      final=SCAN\_HOLD goal=0 fail=0 timeout=0 gates[M=0 T=1 C=1 E=1 K=1 U=1 S=1 MAP=0 X=1] pos=0.0259 att=1.0632 unknown=0 coll=0 geo=0 extra=1 inv=2 suppress=0 scans=90
      recovery[episodeInv=1 episodeBound=0 conservativeFallback=0]
      unreachableReason=PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET

Error using [**assert**](matlab\:matlab.lang.internal.introspective.errorDocCallback\('assert'\))
One or more historical v1.0.0 recoverable failures remain. Full 60-run matrix not started.

Error in [**validate\_S2\_5\_all**](<matlab\:matlab.lang.internal.introspective.errorDocCallback('validate_S2_5_all', '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_2_parallel_candidate_r2/s2_5/validation/validate_S2_5_all.m', 44)>) ([line 44](<matlab: opentoline('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_2_parallel_candidate_r2/s2_5/validation/validate_S2_5_all.m',44,0)>))
assert(all(histPass),'S2\_5\:HistoricalRecoveryFailure', ...
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Error in [**run\_validate\_S2\_5\_all**](<matlab\:matlab.lang.internal.introspective.errorDocCallback('run_validate_S2_5_all', '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_2_parallel_candidate_r2/run_validate_S2_5_all.m', 5)>) ([line 5](<matlab: opentoline('/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_5_estimation_perception_robustness_v1_0_2_parallel_candidate_r2/run_validate_S2_5_all.m',5,0)>))
gate=validate\_S2\_5\_all();
     ^^^^^^^^^^^^^^^^^^^