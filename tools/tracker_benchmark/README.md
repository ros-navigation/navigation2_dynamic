# Tracker benchmark on AGHRI

Scores `kf_hungarian_tracker` on real data: [AGHRI](https://doi.org/10.24385/lincoln.32982638), an agricultural human-robot interaction dataset from the University of Lincoln (CC BY 4.0). A robot with a Livox Mid-360 3D lidar records people walking, carrying, pushing and picking. Every lidar scan (10 Hz) has human-annotated 3D boxes with person IDs, and the robot's TF (`map -> odom -> base_link`) is recorded. The scores come from the evaluator of the [AGHRI benchmark](https://github.com/LCAS/AGHRI-dataset-benchmark) (Apache 2.0), the same one used for its published AB3DMOT, CenterPoint and SimpleTrack results.

## How it works

For each sequence, a fresh tracker node is created and its `callback()` is called once per scan, without an executor, so a run is deterministic and takes seconds. The boxes are moved from the lidar frame into `map` with the recorded pose at the scan's stamp. The published obstacles are moved back into the lidar frame of the same scan and written as MOT3D files, which the AGHRI evaluator matches to the annotations at oriented BEV IoU 0.25.

- **Detections**: by default the annotated boxes with the identities withheld (the benchmark's "ground truth as detections" protocol), which measures the tracking alone. `--detections` takes the output of a real detector instead (see [Real detector](#real-detector-optional)).
- **Publication**: every track is published (`vel_filter` and `height_filter` are opened), because AGHRI also annotates people who stand still. `--publish-default` keeps the node's filters.
- **Detector delay**: `--tf-latency L` gives the tracker lidar-frame boxes and only the TF recorded up to stamp + L, so the node looks the transform up itself, as on a robot whose detector takes L seconds. When the tracker looks the transform up at the detection stamp, L has to be above 0, because at L = 0 no transform after the stamp exists yet.
- **Parameters**: `--param NAME:=VALUE` sets a tracker parameter as on the ROS command line, for example `--param 'measurement_noise_cov:=[0.01,0.01,0.01]'`.
- **Tracker code**: the package in this repository, from source (`--tracker-src` selects another one). Only `nav2_dynamic_msgs` has to be built.

## Setup

1. **Data.** Download the archives from the [DOI page](https://doi.org/10.24385/lincoln.32982638) (`dataset_part1.zip` to `dataset_part10.zip`, about 70 GB; `dataset_summary.csv` lists the archive of each sequence). The `test` split needs parts 1, 2, 6, 8 and 10, `val` needs 1, 2, 3, 8 and 10, `moving` needs all ten. Without a detector only the annotations and the TF are used (a few MB per archive):

   ```bash
   for f in dataset_part*.zip; do unzip -q "$f" '*/annotations/lidar_ann.json' '*/metadata/tf/*' -d ~/aghri; done
   ```

   The detector also needs `'*/sensor_data/lidar/*'`.

2. **AGHRI benchmark** (evaluator, split lists; tested at commit `c042f03`):

   ```bash
   git clone https://github.com/LCAS/AGHRI-dataset-benchmark ~/AGHRI-dataset-benchmark
   ```

3. **Python with `motmetrics`** for the evaluator. A venv that also sees the system packages runs both the tracker and the evaluator:

   ```bash
   python3 -m venv --system-site-packages ~/aghri_venv
   ~/aghri_venv/bin/pip install motmetrics
   ```

4. Build and source this workspace (for `nav2_dynamic_msgs`).

## Run

```bash
source install/setup.bash
~/aghri_venv/bin/python src/navigation2_dynamic/tools/tracker_benchmark/aghri_benchmark.py \
  --data ~/aghri --aghri-benchmark ~/AGHRI-dataset-benchmark --split test --out /tmp/aghri_test
```

```
6 sequences (mean over sequences): MOTA 0.683  IDF1 0.665  HOTA 0.535  |  IDSW 19  FP 614  FN 550
people: 16, first published after 0.00 s (median), never published 0  |  spurious boxes 0  |  dropped person-frames 550 of 3227
```

The output folder holds `metrics.csv` (per sequence, written by the AGHRI evaluator), `summary.json` (the numbers above and the parameters used), `tracks/` (the published obstacles) and `gt/` (the annotations in MOT3D format, converted with the benchmark's own code).

**Splits**: `val` (7 sequences) and `test` (6) are the benchmark's held-out lists. `moving` ([splits/moving.txt](splits/moving.txt)) has the 29 sequences whose "Robot movements" is "move" in `dataset_summary.csv`. A file with sequence names also works.

**Metrics**: MOTA, IDF1 and HOTA are means over the sequences (the "macro" rows that the AGHRI benchmark reports); IDSW, FP and FN are sums. Three more numbers describe the people:

- *first published after*: time from a person's first annotation to the first published box that overlaps them;
- *spurious boxes*: published boxes of tracks that never overlap an annotated person (AGHRI annotates only people, so some may be other real objects);
- *dropped person-frames*: annotated person-frames without a published box after that person was first published.

## Results

Tracker of this repository at `acf668d`.

**Test split, annotated boxes as detections:**

| tracker | MOTA | IDF1 | HOTA | IDSW |
|---|---|---|---|---|
| kf_hungarian_tracker, default parameters | 0.683 | 0.665 | 0.535 | 19 |
| kf_hungarian_tracker, `measurement_noise_cov: [0.01, 0.01, 0.01]` | 0.980 | 0.856 | 0.798 | 16 |
| AB3DMOT \* | 0.940 | 0.652 | 0.707 | 28 |
| CenterPoint tracker \* | 0.976 | 0.859 | 0.889 | 15 |
| SimpleTrack \* | 0.966 | 0.721 | 0.755 | 31 |

\* published by the AGHRI benchmark: `3d-tracking/reports/heldout/test/pointpillars_scratch/summary.csv`, source `gt`, macro rows.

With the default `measurement_noise_cov` of 1 m² the filter follows a walking person too slowly for a lidar with centimetre noise: of the 550 missed person-frames, 543 have a published box less than 0.5 m away (median 0.2 m), too far off for IoU 0.25. With `measurement_noise_cov` 0.01, association (AssA 0.812) is close to the CenterPoint tracker (0.819); the gap in HOTA is localisation (DetA 0.832 vs 0.980), because the CenterPoint tracker publishes the detection boxes unchanged (its MOTP IoU is 1.000) while this tracker publishes filtered positions.

**Test split, PointPillars detections** (score ≥ 0.25; the threshold and the two parameters were chosen on `val`):

| parameters | MOTA | IDF1 | HOTA | IDSW | spurious boxes |
|---|---|---|---|---|---|
| default | 0.602 | 0.570 | 0.410 | 34 | 42 |
| `measurement_noise_cov: [0.01, 0.01, 0.01]`, `death_threshold: 6` | 0.898 | 0.827 | 0.601 | 10 | 43 |

**Moving split, annotated boxes, `measurement_noise_cov: [0.01, 0.01, 0.01]`, detector delay** (`--tf-latency`):

| detector delay | none (exact pose) | 0.1 s | 0.2 s | 0.3 s |
|---|---|---|---|---|
| HOTA | 0.814 | 0.768 | 0.689 | 0.617 |
| MOTA | 0.973 | 0.970 | 0.953 | 0.850 |

The tracker transforms the detections with the latest transform, not the one at their stamp, so while the robot turns a delayed detection is moved to the wrong place (#39).

## Real detector (optional)

`aghri_detect.py` runs the PointPillars checkpoint fine-tuned on AGHRI that the benchmark repository ships (stored with Git LFS: `git lfs pull --include late_fusion/deepfusionmot/checkpoints/aghri_pointpillars_finetune`), with the model config stored in the checkpoint. It writes one JSON per scan in the AGHRI export format, `<out>/<sequence>/<pcd stem>.json`. It needs the lidar scans and an mmdetection3d environment instead of the ROS one (tested with Python 3.10, torch 2.1.2, mmcv 2.1.0, mmdet 3.3.0, mmdet3d 1.4.0, mmengine 0.10.7); on a CPU it takes about 0.06 s per scan.

```bash
~/mmdet3d_venv/bin/python src/navigation2_dynamic/tools/tracker_benchmark/aghri_detect.py \
  --data ~/aghri --aghri-benchmark ~/AGHRI-dataset-benchmark --split test --out ~/aghri_dets
~/aghri_venv/bin/python src/navigation2_dynamic/tools/tracker_benchmark/aghri_benchmark.py \
  --data ~/aghri --aghri-benchmark ~/AGHRI-dataset-benchmark --split test --out /tmp/aghri_pp \
  --detections ~/aghri_dets --min-score 0.25
```

The detector-based results of the AGHRI benchmark use a PointPillars trained from scratch whose checkpoint is not in the repository, so they are not comparable with the numbers above.

## Limits

- AGHRI has only people, recorded on a farm (crop rows and footpaths), and the robot is slow (mostly under 1 m/s).
- The callback is called directly: executor timing, message loss and CPU load are not part of the result. Only the detector delay can be simulated, with `--tf-latency`.
- Not a `colcon test`; nothing here runs in CI.
