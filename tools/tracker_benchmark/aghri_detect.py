#!/usr/bin/env python3
# Copyright 2026 A Gudamu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional: AGHRI PointPillars detections for aghri_benchmark.py. See README.md."""

import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import types

CHECKPOINT = ('late_fusion/deepfusionmot/checkpoints/aghri_pointpillars_finetune/cluster_run/'
              'weights/best.pth')
MMDET3D = '3d-detection/benchmarks/mmdetection3d'
SCORE_FLOOR = 0.05  # aghri_benchmark.py --min-score sets the threshold


def open3d_stub():
    """open3d is imported by the AGHRI code but only used for visualisation."""
    o3d = types.ModuleType('open3d')
    for sub in ('geometry', 'utility', 'visualization'):
        m = types.ModuleType('open3d.' + sub)
        setattr(o3d, sub, m)
        sys.modules['open3d.' + sub] = m
    o3d.visualization.Visualizer = type('_Placeholder', (), {})
    sys.modules['open3d'] = o3d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True, type=Path)
    ap.add_argument('--aghri-benchmark', required=True, type=Path)
    ap.add_argument('--split', default='test')
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--device', default='cpu')
    a = ap.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from aghri_benchmark import find_scene, scan_frames, split_names

    bench = a.aghri_benchmark
    sys.path.insert(0, str(bench / MMDET3D))          # aghri3d
    sys.path.insert(0, str(bench / MMDET3D / 'src'))  # pcd -> bin converter
    open3d_stub()
    import torch
    from mmdet3d.apis import inference_detector, init_model
    from mmengine.config import Config
    from prepare_aghri_lidar_dataset import _convert_pcd_to_bin

    ckpt = bench / CHECKPOINT
    cfg_text = torch.load(str(ckpt), map_location='cpu')['meta']['cfg']
    with tempfile.TemporaryDirectory() as tmp:
        cfg_file = Path(tmp) / 'pointpillars_aghri_finetune.py'
        cfg_file.write_text(cfg_text)
        model = init_model(Config.fromfile(str(cfg_file)), str(ckpt), device=a.device)
        model.eval()
        for name in split_names(a.split, bench):
            scene = find_scene(a.data, name)
            out = a.out / name
            out.mkdir(parents=True, exist_ok=True)
            t0, n = time.time(), 0
            for _, stem in scan_frames(scene):
                dst = out / f'{stem}.json'
                if dst.exists():
                    continue
                points = Path(tmp) / f'{stem}.bin'
                _convert_pcd_to_bin(scene / 'sensor_data' / 'lidar' / f'{stem}.pcd', points)
                with torch.no_grad():
                    res = inference_detector(model, str(points))
                points.unlink()
                res = res[0] if isinstance(res, (list, tuple)) else res
                pred = res.pred_instances_3d
                boxes, scores = pred.bboxes_3d.tensor.cpu().numpy(), pred.scores_3d.cpu().numpy()
                dets = [dict(x=float(x), y=float(y), z=float(z), l=float(ln), w=float(w),
                             h=float(h), yaw=float(yaw), score=float(s), label=0)
                        for (x, y, z, ln, w, h, yaw), s in zip(boxes[:, :7], scores)
                        if s >= SCORE_FLOOR]
                dst.write_text(json.dumps(dets))
                n += 1
            print(f'{name}: {n} scans, {time.time() - t0:.0f} s', flush=True)


if __name__ == '__main__':
    main()
