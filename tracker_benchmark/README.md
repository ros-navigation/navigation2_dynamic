# tracker_benchmark

Ground-truth evaluation of `kf_hungarian_tracker`: a scripted scene with a moving robot, walking
people and static objects, a simulated detector, and a scorer that compares `/tracking` with the truth.
No simulator and no sensor data are needed; a run takes a minute.

It answers questions such as: how well are moving people covered, how large are the position and
velocity errors, how often does a person get a new id, how often is a static object published as a
dynamic one, and what happens when the robot turns or the detector is slow.

## What is in here

| file | purpose |
|---|---|
| `scenario_node.py` | robot motion in `odom` (`static`, `straight`, `spin`, `arc`), 6 people walking between random waypoints (0.5–1.6 m/s, some pause 2–6 s), 6 static objects. Publishes `/detection` in `base_link` (position noise, missed detections, false positives, limited range, processing delay, velocity left at 0), `/truth` in `odom`, and `odom -> base_link` on `/tf` |
| `eval_node.py` | matches every `/tracking` message with the truth of the same stamp (1 m gate) and writes a JSON summary |
| `run_bench.sh` | starts tracker + scorer + scenario for one condition: `run_bench.sh <tag> <profile> <ros_domain_id> <tracker_params.yaml> [B_DELAY=0.3 B_PMISS=0.3 ...]` |
| `offline_eval.py` | same scene and metrics without an executor: the tracker node is created and its callback is called directly with detections already in the global frame. Deterministic, 20 seeds x 60 s in under a minute: `python3 offline_eval.py --seeds 20 [--pmiss 0.3] [--noise 0.15] [--set death_threshold=10 ...]` |

Scenario settings are environment variables (`B_PROFILE`, `B_SEED`, `B_DURATION`, `B_RATE`, `B_NOISE`,
`B_PMISS`, `B_FP`, `B_RANGE`, `B_DELAY`, `B_PEOPLE`, `B_STATIC`), see the top of `scenario_node.py`.

## Metrics

| metric | meaning |
|---|---|
| coverage / coverage after 1 s | fraction of frames in which a moving person inside the detector range has a published track within 1 m (second number: only people that have been moving and visible for more than 1 s) |
| pos_rmse, vel_rmse | error of the matched track against the true position / velocity |
| id switches per person-minute | how often the id following a person changes while the person stays in range (offline) |
| static reported | fraction of frames in which a static object, or a person standing still, has a published track |
| ghosts | published tracks with no object within 1 m, per message |

## Results with the default parameters

Detector: 10 Hz, 5 cm position noise, 10 % missed detections, 0.2 false positives per frame, 8 m range.
`±` is the standard error over 20 seeds x 60 s (`offline_eval.py`, tracking logic only).

| condition | coverage after 1 s | pos_rmse [m] | vel_rmse [m/s] | id switches / person-min | static reported | ghosts / frame |
|---|---|---|---|---|---|---|
| default parameters | 0.991 | 0.211 ± 0.004 | 0.531 ± 0.015 | 6.6 ± 0.4 | 0.117 | 0.033 |
| no misses, no false positives | 0.997 | 0.221 ± 0.005 | 0.540 ± 0.016 | 3.5 ± 0.2 | 0.112 | 0.023 |
| 30 % missed detections | 0.917 | 0.195 ± 0.003 | 0.563 ± 0.015 | 18.5 ± 0.6 | 0.163 | 0.026 |
| 30 % missed, `death_threshold: 10` | 0.991 | 0.226 ± 0.004 | 0.557 ± 0.014 | 7.0 ± 0.4 | 0.133 | 0.295 |
| `measurement_noise_cov: 0.01` (matches the 5–10 cm noise) | 0.997 | 0.067 ± 0.001 | 0.322 ± 0.010 | 2.7 ± 0.3 | 0.387 | 0.023 |
| 15 cm position noise | 0.991 | 0.216 ± 0.003 | 0.552 ± 0.015 | 10.0 ± 0.6 | 0.201 | 0.078 |

What these numbers say:

* **The default `measurement_noise_cov` of 1 m² makes the filter slow.** With 5 cm of real noise the
  position error is 0.21 m and the velocity error 0.5 m/s. With the value matched to the detector
  both drop (0.07 m, 0.32 m/s).
* **Ids change mostly when a person changes velocity.** In a people-only run without misses or false
  positives (10 seeds x 60 s), 171 of 215 id changes happened within 1.5 s of a stop, start or turn: the slow filter overshoots,
  the detection falls outside `cost_filter` (1 m), the track dies after 3 frames and a new one is born.
* **Static objects are published as dynamic ones 10–40 % of the time.** The velocity estimate is
  noisy and `vel_filter[0]` is a hard threshold on a single frame. A faster filter makes this worse
  (0.39), because its velocity estimate is noisier.
* **`death_threshold` trades id switches for ghosts** (old tracks coast on while a new one is born).

## Robot rotation and detector delay (live nodes, `run_bench.sh`)

`spin`: the robot turns in place at 0.6 rad/s, direction reversed every 8 s. Mean of 3 seeds x 60 s.

| detector delay | pos_rmse [m] | static reported | ghosts / message | coverage after 1 s |
|---|---|---|---|---|
| 0 s | 0.19 | 0.10 | 0.02 | 0.99 |
| 0.1 s | 0.37 | 0.32 | 0.08 | 0.97 |
| 0.3 s | 0.62 | 0.23 | 1.60 | 0.66 |

The tracker transforms detections with the latest transform instead of the one at the detection
stamp, so a delay turns robot rotation into apparent obstacle motion.

## Limits

* The detector is simulated: independent Gaussian position noise, independent misses, uniform false
  positives. No occlusion, no merged or split detections, no size or orientation noise.
* People are points that walk straight between waypoints and do not avoid each other or the robot.
* This scores the tracker only; nothing downstream (costmap, controller) is evaluated.
