# Frozen Parent Contract — S2.5 v1.0.9

This package is the validated S2.5 parent consumed by S2.6.

- Treat the validated S2.5 runtime/autonomy source as immutable.
- Do not weaken safety, estimator, mapping, trajectory, stopping, timeout, route-freshness, inflation, or unknown-space gates to satisfy downstream integration.
- Preserve frozen S2.4-G and S2.3 parent byte identity.
- If S2.6/SITL exposes a genuine defect, fork a new child version; do not patch this frozen parent in place.
- Retain the archived MATLAB 71/71 qualification log as release evidence.
