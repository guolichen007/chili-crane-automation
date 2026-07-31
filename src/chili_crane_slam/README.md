# chili_crane_slam

Phase 0 defines dual-LiDAR, runtime-mode, and servo-prior contracts only.

Planned extraction order:

1. consume-once merger and typed diagnostics;
2. generic semantic/dynamic registration masks;
3. observability and fitness circuit breaker;
4. servo-seeded NDT and pose fusion;
5. rail-aware relocalization and immutable map snapshots.

Every adapted file must record upstream path and SHA. No NDT implementation has
yet been claimed to build in this repository.
