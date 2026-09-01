# S2.3 replay-first next step

This package is diagnostic instrumentation only. It intentionally preserves
all current S2.3 mapping, planning, control and acceptance behaviour.

It adds capture of every accepted raw LiDAR/depth packet, the exact estimated
pose and mapper call time used for insertion, and a production-mapper replay
function that demands exact agreement with the coupled-flight final map.

Required sequence:

1. Run `unknown_room_nominal`, seed 0 once with this package.
2. Run `replay_perception_log_S2_3` on the new MAT file.
3. Do not apply boundary or static-evidence corrections unless replay returns
   PASS and reproduces the current false-free and recall metrics exactly.
4. Apply candidate policies to that same packet stream and use the production
   validator before transferring them into the coupled flight loop.

The expected coupled result remains FAIL at this diagnostic stage; changed
mission behaviour would indicate unintended instrumentation impact.
