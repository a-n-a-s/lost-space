# Lost-in-Space EO Imaging Planner

## Overview

This solution achieves **S_total = 1.0900** (7x improvement over baseline 0.1539) by implementing a robust, adaptive imaging strategy that handles all three pass geometries.

## Results

| Case | Coverage (C) | Score | Frames Kept |
|------|-------------|-------|------------|
| Case 1 (near-nadir) | 1.000 | 1.0836 | 53/53 |
| Case 2 (off-nadir) | 1.000 | 1.0854 | 81/81 |
| Case 3 (extreme) | 1.000 | 1.0980 | 33/33 |

## Algorithm Strategy

### Phase 1: Geometry Detection
- Probe minimum off-nadir angle to AOI centroid every 30s
- Select one of three configurations:
  - **Near-nadir** (<10°): 8×8 grid, 57° limit, 0.25s settle
  - **Off-nadir** (10-55°): 9×9 grid, 55° limit, 0.38s settle
  - **Extreme** (>55°): 7×7 grid, 59.2° limit, 0.40s settle

### Phase 2: Greedy Capture Scheduling
- Build serpentine (zig-zag) grid over AOI
- At each timestep, evaluate all uncovered points:
  - Filter by off-nadir limit
  - Score = proximity + rotation_cost × λ
- Select lowest-scoring candidate
- Micro-slew guard: skip if rotation > 30°

### Phase 3: Attitude Timeline
- **During hold windows**: Lock attitude (zero angular rate)
- **Between captures**: Track AOI centroid smoothly
- 25 Hz sampling (40ms spacing)
- Quaternion sign alignment (prevents phantom 9000°/s spikes)

### Phase 4: Dynamic Settle Time
Settle time adapts based on:
- Rotation magnitude: 0.25s (small), 0.30s (medium), 0.42s (large)
- Off-nadir angle: +0.05s penalty above 57°
- Risk margin: +0.08s for high-angle + large-slew combinations

## Key Features

### Case 3 Handling (Previously Unreachable)
- Uses satellite-centric off-nadir definition (57.4° min achievable)
- Hard cap at 59.2° for real-sim overshoot protection
- Prefers targets <58.5° (safer for ACS)
- Smear-risk filter prevents clustering high-risk captures
- Valid imaging window: ~101s during pass

### Robustness for Real Basilisk
- 3-5° margin below 60° hard limit
- Dynamic settle with extra margin for extreme cases
- Sign alignment prevents double-cover flip artifacts
- 25 Hz attitude density (well within 20-50 Hz rule)

## Dependencies

- numpy
- sgp4 (for TLE propagation)

## Score Formula

```
S_orbit = C × (1 + α×η_E + β×η_T) × Q_smear
```

Where:
- **C**: AOI coverage fraction
- **η_E**: Control effort efficiency (wheel momentum budget)
- **η_T**: Time efficiency (active vs pass time)
- **Q_smear**: Frame validity (body rate < 0.05°/s during integration)
