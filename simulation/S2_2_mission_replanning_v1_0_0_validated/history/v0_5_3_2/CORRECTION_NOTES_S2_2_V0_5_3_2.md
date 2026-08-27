# S2.2 v0.5.3.2 correction

The v0.5.3.1 seed-7 run showed that the lifecycle completed but minimum wall
clearance became negative. The brake pulse itself was constructed from the
last aid-bounded velocity, but two post-loss velocity paths remained active:

1. explicit ESKF velocity damping during the pulse; and
2. the geometric controller speed guard, which was still evaluated during
   blind horizontal mode and could create acceleration from a drifting ESKF
   velocity during descent.

v0.5.3.2 removes both paths. It applies one frozen feedforward impulse, then
commands zero horizontal acceleration for 0.16 s so the jerk-limited command
returns to zero, and only then starts the vertical emergency descent. The
normal speed guard remains active for all observable navigation modes.

No safety or validation threshold is changed. MATLAB execution remains the
authoritative coupled 6-DOF test.
