# Recorded trace coverage

## Available full raw replay trace

The compatible final full trace available during package generation is the
release `UNKNOWN_ROOM_NOMINAL` trial. It contains 1,342 captured raw
LiDAR/depth records and 81 stored map snapshots. It was replayed through the
S2.4 A-D reference stack twice with identical digests. Incremental frontier
extraction matched full extraction for every snapshot, and no unsafe viewpoint
was accepted.

The nominal trace produced zero accepted observation viewpoints under the exact
0.602 m physical-and-unknown inflation mask. This is not converted into a
synthetic event or treated as a failure: S2.3 had already completed its own safe
scan behavior, while the 15-scenario contract catalogue separately exercises
accepted target-relevant viewpoints and ranking.

## Difficult scenarios

The uploaded source release contains final validation summaries and aggregate
release evidence, but it does not include individual final full raw
perception/map replay MAT files for every difficult S2.3 scenario. Several older
MAT artifacts inspected from the file library used incompatible pre-release
schemas; they are not silently accepted as final evidence.

Consequently, difficult-scenario mechanisms are covered here by the combined
15-scenario Python geometry/contract matrix. Full raw difficult-trace replay
remains an explicit MATLAB/offline gate when compatible final MAT traces are
available. The package must not claim that gate passed before those traces are
run.
