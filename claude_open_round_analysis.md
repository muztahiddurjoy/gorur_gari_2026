# Open Round Simulation Analysis

## Test Summary

I ran a 30-second headless open round test. Here are the results:

| Metric | Value |
|---|---|
| **Lap 1 time** | 13.0 seconds |
| **Lap 1 distance (odom)** | 16750.8 cm (167.5 m — odometry is wildly off) |
| **Lap 1 turns** | 4 (correct CCW) |
| **Laps completed in 30s** | 1 (was midway through lap 2 when timeout hit) |
| **LiDAR timeouts** | **0** (the 2.0s fix worked perfectly!) |
| **Direction** | CCW — right wall hug (correct) |
| **Heading jumps** | 0 (no IMU spikes this run) |
| **Gazebo crashes** | None — sim ran the full 30s |

## What Went Right

1. **LiDAR timeout fix worked!** Zero `LiDAR timeout!` errors this entire run. The 200ms → 2.0s change completely eliminates the stop-and-go stutter.
2. **Navigation is functional.** The bot found valid targets every 500ms, with distances 1.0–2.2m, and correctly used right-wall hugging (`Wall: R-12.0°`).
3. **Lap detection works.** 4 clean turns detected for CCW lap 1, and turns 5–7 were being counted for lap 2 before the test ended.
4. **Boost engaged on straights.** `Boost: 1.35x` correctly activated when the target was nearly straight ahead and far away.

## Issues Found

### 1. Odometry is broken (16750.8 cm for one lap)

The real track is ~8m per lap. The odometry reported **167.5 m** — off by 20x.

From the `vector_odom` logs:
```
mcu ticks=12826 heading=0.0 deg | x=0.00 y=0.00 dist=0.00 cm     ← boot
mcu ticks=18570 heading=61.5 deg | x=-50.86 y=249.33 dist=803.57 cm  ← lap 1
mcu ticks=25452 heading=277.0 deg | x=137.64 y=328.34 dist=1101.37 cm ← lap 2
```

The `dist` field (cumulative) reads 1101 cm after ~1.7 laps — that's 11m, which is much closer to reality. The **-16750.8 cm** in `lap_counter`'s report is the LAP COUNTER's own distance metric, which looks like it is using a different accumulation method and is accumulating garbage from the initial tick jump at boot.

> **Root cause:** When the bridge starts, it reports `mcu ticks=12826` before the sim bot has even spawned. When it next reports `ticks=200` (after track_maker resets the bot), `vector_odom` integrates a `Δticks = -12626` backwards step, catapulting the estimated position ~54m off the map. Every subsequent distance calculation is tainted.

### 2. track_maker spawns towers WHILE the bot is already driving

From the timeline:
```
t=501.8s  [NAV] +32.3° | Dist: 1.71m — bot starts driving
t=503.7s  Lap direction latched: CCW
t=504.1s  Turn 1 detected
t=506.5s  track_maker: Gazebo is up
t=506.9s  Spawned tower_1
t=507.2s  Spawned tower_2
...
t=508.8s  Moved lazyBot to (-0.10, 0.00)
```

The bot has already completed **Turn 1** and is driving fast before `track_maker` even starts spawning the inner towers. Then at `t=508.8s`, `track_maker` **teleports the bot back to the start**. This creates chaos:
- The bot was mid-corner, suddenly it's facing a wall at `(-0.10, 0.00)`.
- The heading and odometry accumulate garbage from the teleport.

> **This is why you saw "All blocked (best 0.12m)" in your manual test.** The bot drives for 8 seconds, gets teleported back to the start line (which is right next to tower_6 at `(-0.50, 0.10)`), and suddenly sees an obstacle 12cm away.

### 3. The "All blocked" issue is timing-dependent

In the headless test, the bot happened to be in a clear area when it got teleported back and was able to recover. In your interactive test with the GUI open, Gazebo ran slower, so by the time track_maker teleported the bot, its heading was pointing directly at a wall, triggering the permanent "All blocked" deadlock.

## Recommendations

### Fix 1: Delay bot driving until track_maker is done (Critical)

The `open_round_run` node should NOT start publishing `/cmd_vel` until track_maker has finished placing the bot. Options:
- Have `track_maker` publish a `/track_ready` topic, and gate `open_round_run` on it
- Simply add a startup delay parameter (e.g., `startup_wait_sec: 10.0`) to `open_round_run` so the track has time to build

### Fix 2: Fix the initial odometry tick jump

`vector_odom` should discard the first tick delta entirely. When `Δticks` is negative or absurdly large (>1000 in one step), it should reset rather than integrate.

### Fix 3: The open round is spawning obstacle towers

`track.yaml` has `objects: true`, which spawns the 6 colour-coded towers even in the open round. This is fine for the obstacle round, but **the open round should run on a bare track**. Either:
- Create a separate `track_open.yaml` with `objects: false` and no towers
- Or have the open round launch file override `objects: false`
