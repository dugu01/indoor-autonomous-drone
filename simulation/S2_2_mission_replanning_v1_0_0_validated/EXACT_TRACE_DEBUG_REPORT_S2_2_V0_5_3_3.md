# S2.2 v0.5.3.3 exact-trace debug report

## Evidence source

The diagnosis used the full MATLAB v7.3 trial data from
`XY_LOSS_EMERGENCY_LAND`, seed 7, v0.5.3.2. The replay metrics are stored in
`SEED7_EXACT_TRACE_REPLAY_V0_5_3_3.json`.

## Root cause

The horizontal brake was not the remaining failure. At 16.20 s, truth XY
speed was 0.2735 m/s. It fell to 0.0037 m/s near 17.00 s and was 0.0254 m/s
at emergency-descent release.

The vertical profile completed at 21.52 s and the first fully qualified ground
contact occurred at 21.62 s. At that instant:

- estimated altitude: 0.0324 m;
- estimated vertical speed: -0.0063 m/s;
- truth XY position: [3.6053, 1.3577] m;
- raw nearest-wall distance: 1.3577 m.

The old land detector required ten uninterrupted contact samples (0.20 s).
The simple ground model rebounded by a few millimetres, so the first contact
lasted only one sample. The first sufficiently long interval did not occur
until 35.36-35.56 s. Landing recognition was therefore delayed by 13.92 s.

During that delay the estimator reported almost zero roll and pitch while the
truth vehicle retained approximately +0.068 deg roll and -0.058 deg pitch.
This generated mean horizontal acceleration of roughly
[-0.00615, -0.00763] m/s^2 and produced the large wall-directed drift.

## Correction

Touchdown recognition is now armed only after the commanded descent profile is
complete. A contact pulse is accepted only when the selected estimator also
reports near-ground altitude and low vertical speed. That qualified event is
latched, so millimetre-scale model rebound cannot erase a valid touchdown
before the next state-machine step cuts motor command.

No truth state is used by the detector. `packet.groundContact` remains the
explicit simulated onboard contact signal.

## Exact replay

Replaying the package ground dynamics from the first qualified contact with
motor command cut for the 0.60 s disarm interval gives:

- additional XY displacement: 0.0324 m;
- final XY: [3.5830, 1.3342] m;
- minimum raw wall distance: 1.3342 m;
- required wall distance: 0.5020 m;
- remaining wall margin: 0.8322 m.

This replay uses the recorded MATLAB truth state and the exact source ground
update, rather than a randomized point-mass approximation. MATLAB rerun is
still required for authoritative coupled validation.
