# Simulation results (Gazebo Fortress, headless, Docker arm64)

All errors are measured against the simulator's true pose (`/ground_truth/odom`, OdometryPublisher),
never against anything the robot estimates. 8 goals x 2 passes = 16 trials per series, one session
per series (fresh map). Raw data: `data/<series>/` (`trials.csv`, `summary.json`, `samples.csv`,
`gt.tum`, `slam.tum`).

## A. Natural operation (no injected faults)
| Series | World | Guard | Trials ok | Goal err mean / p95 (m) | Return err mean / p95 (m) | ATE RMSE per trial, median (m) | Localization jumps |
|---|---|---|---|---|---|---|---|
| 01 | one texture on all walls (old textures, slow turns 0.06 rad/s) | none | 16/16 reported | 0.28 / 1.56 | 0.27 / 0.66 | 0.06 | 3 trials (ATE 1.3-3.5 m), 1 false success |
| 02 | one texture on all walls (regenerated, turns 0.34 rad/s) | on | 16/16 SUCCESS | 0.21 / 0.27 | 0.23 / 0.46 | 0.05 | none |
| 03 | unique texture per wall | on | 16/16 SUCCESS | 0.20 / 0.25 | 0.23 / 0.33 | 0.05 | none |

Series 01 failures were not reproduced in 02/03. Between 01 and 02 several things changed at once
(wheel friction fix -> faster turns, regenerated textures, new mission manager), so the cause of the
01 jumps (probably false loop closures from identical wall textures) is a hypothesis, not a result.

## B. Injected map->odom jumps (fault_injector.py, ~97 deg rotation of the map frame about its origin)
Odd trials get a jump: idx%4==1 during GOTO, idx%4==3 during RETURN. World: unique textures.

| | Guard ON (series 04) | Guard OFF (series 05) |
|---|---|---|
| Trials finished | 16/16 | 13/16 (3 goto timeouts) |
| Jumps during GOTO: returned home < 0.5 m | 4/4 | 1/4 |
| Jumps during GOTO: reported SUCCESS although not home | 0 | 2/4 (return err 0.84 m and 8.94 m) |
| Jumps during RETURN: returned home < 0.5 m | 4/4 | 3/3 finished |
| Jumps flagged (result != SUCCESS) | 8/8 | n/a (no guard) |
| False alarms in non-injected trials | 0/8 | n/a |
| Trials after the first unrecovered jump | all fine | trials 6-15: 3 timeouts, 7 ABORTED_GOAL_UNREACHABLE (robot stuck far from start) |

Results of the guard-OFF series after trial 5 are a cascade (the robot never recovered), not
independent samples.

## Caveats
- Simulation with ideal camera/IMU; one arena; 16 trials per series, single run each.
- The injected fault is synthetic. It rotates the map frame about its origin; start poses are close
  to the origin, so the RETURN-phase jump barely moves the home target. GOTO-phase jumps are the
  harmful case. In guard-ON GOTO trials the return error is ~0 mostly because the abort came
  within ~3 s of leaving the start.
- The guard (thresholds 0.4 m / 0.2 rad per check) saw zero false alarms in 40 non-injected trials,
  but none of them contained a legitimate large loop-closure correction, so that is untested.
- The `false_success` / `flagged_but_fine` counters in `summary.json` are threshold artefacts in
  borderline trials (goal error 0.30 m; a jump too small to exceed the 1 m ATE bar), not failures.
  In guard-OFF `flagged_by_guard` counts Nav2 aborts, not guard actions.

## D. Saved map + localization mode (same world, goals and guard as series 03)
Series 06: the 16-trial series was run in mapping mode, RTAB-Map was stopped gracefully (SIGINT) so it
wrote the database (327 MB), then the same 16 trials were run in `mode:=localization` on that database.

| | S03 fresh map | S06 mapping | S06 localization (saved map) |
|---|---|---|---|
| Result | 16/16 SUCCESS | 16/16 SUCCESS | 15 SUCCESS + 1 COMPLETED_JUMP_DETECTED |
| Goal err mean / p95 (m) | 0.20 / 0.25 | 0.19 / 0.27 | 0.21 / 0.25 |
| Return err mean / p95 (m) | 0.23 / 0.33 | 0.24 / 0.35 | 0.22 / 0.30 |
| ATE RMSE per trial, median (m) | 0.05 | 0.04 | 0.03 |
| Max SLAM error in the run (m) | 0.25 | 0.18 | 4.82 |

- Localization mode works from a saved database and navigates with the same accuracy as with a fresh map.
  In these short-range goals the saved map gave no accuracy gain; its value (known layout from the first
  second, goals outside what the camera has just seen) was not demonstrated: the saved map only covers the
  area visited by these goals, and a tour of the whole arena failed (section C).
- Trial 6 had a natural (not injected) localization jump during RETURN: map->odom moved 5.6 m / 90 deg, twice
  (away and back, ~10 s apart). The guard flagged it (`COMPLETED_JUMP_DETECTED`) and the robot still
  came home within 0.29 m of the start. It is a single event in 16 trials, so it says "this happens, and
  the guard caught the one that did", not anything about rates.
- The database is large (327 MB for ~20 min of driving); there is no map-saving service yet, the database is
  written only when RTAB-Map exits cleanly.

## C. Open problem: long exploratory tours (not solved)
Attempt: build a map of the whole arena by touring 10 waypoints (`tools/survey.py`, steps of 2 m),
to then run far goals on the saved map. Result: 2/10 waypoints reached, then the map became unusable.
What the data showed (one stack log of ~27 min, unique-texture world, guard on):
- Stereo visual odometry lost tracking in 48 episodes; three of them lasted 398 s, 178 s and 772 s.
  After a loss it does not recover until the camera sees the old map again, while the robot keeps moving.
- The map-frame robot pose jumped by ~4 m at one point; RTAB-Map rejected all later loop closures, and
  the occupancy grid filled with false obstacles, so Nav2 could no longer plan even short steps.
- Stereo ground speckle (5 cm ground threshold at 3-6 m range) also adds false obstacles; a 0.2 m
  threshold plus noise filter did not fix the tour problem on its own.
- `Odom/ResetCountdown` (auto-reset of the odometry) made things worse: the reset takes its pose from the
  `odom->base_link` TF, which the EKF publishes only after it receives odometry, so startup deadlocked.
  Reverted. Neither the grid changes nor the reset are part of the documented configuration.
Not investigated: why tracking is lost (candidates: recovery behaviours spinning/backing, collisions
tilting the robot against the planar `Reg/Force3DoF` constraint, textureless views), IMU-aided
odometry, a visual-inertial pipeline (ORB-SLAM3 stereo-inertial / cuVSLAM) instead of RTAB-Map's F2M.
The mission guard watches `map->odom` only; an `odom->base_link` jump (odometry restart) is not detected.
