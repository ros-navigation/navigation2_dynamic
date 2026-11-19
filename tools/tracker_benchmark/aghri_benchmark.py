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

"""Benchmark kf_hungarian_tracker on the AGHRI dataset. See README.md."""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
# base_link -> front_lidar_link, from AGHRI calibration/extrinsics.json
BASE_TO_LIDAR = (0.615, 0.280, 0.330)
IOU = 0.25  # as in the AGHRI evaluator


def load_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def yaw_of(q):
    return math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1.0 - 2.0 * (q['y'] ** 2 + q['z'] ** 2))


class LidarPose:
    """Lidar pose in the map frame, interpolated from the recorded TF."""

    def __init__(self, meta):
        mo = load_jsonl(meta / 'tf' / 'map__to__odom.jsonl')
        ob = load_jsonl(meta / 'tf' / 'odom__to__base_link.jsonl')
        self.mo_t = np.array([r['t'] for r in mo])
        self.mo_x = np.array([r['tr']['x'] for r in mo])
        self.mo_y = np.array([r['tr']['y'] for r in mo])
        self.mo_yaw = np.unwrap([yaw_of(r['q']) for r in mo])
        self.ob_t = np.array([r['t'] for r in ob])
        self.ob_x = np.array([r['tr']['x'] for r in ob])
        self.ob_y = np.array([r['tr']['y'] for r in ob])
        self.ob_yaw = np.unwrap([yaw_of(r['q']) for r in ob])
        self.origin = (self.mo_x[0], self.mo_y[0])

    def at(self, t):
        tx = np.interp(t, self.mo_t, self.mo_x) - self.origin[0]
        ty = np.interp(t, self.mo_t, self.mo_y) - self.origin[1]
        ymo = np.interp(t, self.mo_t, self.mo_yaw)
        ox, oy = np.interp(t, self.ob_t, self.ob_x), np.interp(t, self.ob_t, self.ob_y)
        yob = np.interp(t, self.ob_t, self.ob_yaw)
        c, s = math.cos(ymo), math.sin(ymo)
        bx, by, byaw = tx + c * ox - s * oy, ty + s * ox + c * oy, ymo + yob
        c, s = math.cos(byaw), math.sin(byaw)
        lx = bx + c * BASE_TO_LIDAR[0] - s * BASE_TO_LIDAR[1]
        ly = by + s * BASE_TO_LIDAR[0] + c * BASE_TO_LIDAR[1]
        return lx, ly, BASE_TO_LIDAR[2], byaw


def stamp_of(t):
    from builtin_interfaces.msg import Time
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)


def tf_msg(t, parent, child, x, y, q):
    from geometry_msgs.msg import TransformStamped
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    m.header.stamp = stamp_of(t)
    m.transform.translation.x, m.transform.translation.y = float(x), float(y)
    m.transform.rotation.x, m.transform.rotation.y = q['x'], q['y']
    m.transform.rotation.z, m.transform.rotation.w = q['z'], q['w']
    return m


class TfFeeder:
    """Feeds the recorded TF into the tracker's buffer up to a given time."""

    def __init__(self, meta, origin, buffer):
        rows = []
        for r in load_jsonl(meta / 'tf' / 'map__to__odom.jsonl'):
            rows.append((r['t'], tf_msg(r['t'], 'map', 'odom', r['tr']['x'] - origin[0],
                                        r['tr']['y'] - origin[1], r['q'])))
        for r in load_jsonl(meta / 'tf' / 'odom__to__base_link.jsonl'):
            rows.append((r['t'], tf_msg(r['t'], 'odom', 'base_link', r['tr']['x'], r['tr']['y'],
                                        r['q'])))
        rows.sort(key=lambda r: r[0])
        self.rows, self.i, self.buffer = rows, 0, buffer
        st = tf_msg(rows[0][0], 'base_link', 'front_lidar_link', BASE_TO_LIDAR[0],
                    BASE_TO_LIDAR[1], {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
        st.transform.translation.z = BASE_TO_LIDAR[2]
        buffer.set_transform_static(st, 'aghri')

    def advance(self, now):
        while self.i < len(self.rows) and self.rows[self.i][0] <= now:
            self.buffer.set_transform(self.rows[self.i][1], 'aghri')
            self.i += 1


def scan_frames(scene):
    """(stamp, pcd stem) of each scan, in the order of the AGHRI GT files."""
    frames = sorted(json.loads((scene / 'annotations' / 'lidar_ann.json').read_text()),
                    key=lambda f: f['Timestamp'])
    return [(float(f['Timestamp']), Path(f['File']).stem) for f in frames
            if str(f.get('File', '')).endswith('.pcd')]


def run_scene(tracker_cls, scene, det_dir, min_score, tf_latency, out_csv):
    """Runs a fresh tracker over one sequence and writes its output as MOT3D."""
    from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray

    pose = LidarPose(scene / 'metadata')
    node = tracker_cls()
    feeder = None
    if tf_latency is not None:
        feeder = TfFeeder(scene / 'metadata', pose.origin, node.tf_buffer)
    out = []
    node.tracker_obstacle_pub.get_subscription_count = lambda: 1
    node.tracker_obstacle_pub.publish = out.append
    node.tracker_marker_pub.get_subscription_count = lambda: 0

    ids, rows = {}, []
    for fid, (t, stem) in enumerate(scan_frames(scene), 1):
        boxes = json.loads((det_dir / f'{stem}.json').read_text())
        boxes = [d for d in boxes if d['score'] >= min_score]
        lx, ly, lz, lyaw = pose.at(t)
        c, s = math.cos(lyaw), math.sin(lyaw)
        msg = ObstacleArray()
        msg.header.frame_id = 'map' if feeder is None else 'front_lidar_link'
        msg.header.stamp = stamp_of(t)
        for d in boxes:
            o = Obstacle()
            o.size.x, o.size.y, o.size.z = d['l'], d['w'], d['h']
            o.score = float(d['score'])
            if feeder is None:  # map frame, recorded pose
                o.position.x = lx + c * d['x'] - s * d['y']
                o.position.y = ly + s * d['x'] + c * d['y']
                o.position.z = lz + d['z']
                yaw = d['yaw'] + lyaw
            else:  # lidar frame, the tracker looks up the TF
                o.position.x, o.position.y, o.position.z = d['x'], d['y'], d['z']
                yaw = d['yaw']
            o.orientation.z, o.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.obstacles.append(o)
        out.clear()
        if feeder is not None:
            feeder.advance(t + tf_latency)
        node.callback(msg)
        for o in (out[0].obstacles if out else []):
            tid = ids.setdefault(bytes(o.uuid.uuid), len(ids) + 1)
            dx, dy = o.position.x - lx, o.position.y - ly
            x, y = c * dx + s * dy, -s * dx + c * dy
            q = o.orientation
            yaw = yaw_of({'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}) - lyaw
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            rows.append(f'{fid},{tid},{x:.4f},{y:.4f},{o.position.z - lz:.4f},'
                        f'{o.size.x:.4f},{o.size.y:.4f},{o.size.z:.4f},{yaw:.4f},1.0000\n')
    node.destroy_node()
    out_csv.write_text('frame_id,track_id,x,y,z,l,w,h,yaw,score\n' + ''.join(rows))
    return fid, len(ids)


def people_metrics(scene, gt_csv, pred_csv, load_mot3d, iou_matrix):
    """Publication delay of people, spurious boxes and dropped person-frames."""
    stamps = [t for t, _ in scan_frames(scene)]
    gt, pr = load_mot3d(gt_csv), load_mot3d(pred_csv)
    first_gt, first_cov, covered, matched = {}, {}, set(), {}
    for f in sorted(set(gt) | set(pr)):
        g = gt.get(f, np.zeros((0, 9)))
        p = pr.get(f, np.zeros((0, 9)))
        for gid in g[:, 0].astype(int):
            first_gt.setdefault(gid, f)
        hit = set()
        if len(g) and len(p):
            iou = iou_matrix(g[:, 1:8], p[:, 1:8])
            for i, j in zip(*linear_sum_assignment(-iou)):
                if iou[i, j] >= IOU:
                    hit.add(j)
                    first_cov.setdefault(int(g[i, 0]), f)
                    covered.add((f, int(g[i, 0])))
        for j, tid in enumerate(p[:, 0].astype(int)):
            matched[(f, tid)] = matched.get((f, tid), False) or j in hit
    tracks = {}
    for (f, tid), ok in matched.items():
        tracks.setdefault(tid, []).append(ok)
    spurious = sum(len(v) for v in tracks.values() if not any(v))
    dropped = sum(1 for f, g in gt.items() for gid in g[:, 0].astype(int)
                  if gid in first_cov and f > first_cov[gid] and (f, gid) not in covered)
    delays = [stamps[first_cov[gid] - 1] - stamps[f0 - 1] for gid, f0 in first_gt.items()
              if gid in first_cov]
    never = sum(1 for gid in first_gt if gid not in first_cov)
    return dict(delays=delays, never=never, spurious=spurious, dropped=dropped,
                person_frames=sum(len(g) for g in gt.values()))


def split_names(split, bench):
    if split in ('val', 'test'):
        path = bench / '3d-tracking' / 'reports' / 'heldout' / 'split_lists' / f'{split}.txt'
    elif (HERE / 'splits' / f'{split}.txt').exists():
        path = HERE / 'splits' / f'{split}.txt'
    else:
        path = Path(split)
    return [n for n in path.read_text().split() if n]


def find_scene(data, name):
    found = [p for p in list(data.glob(name)) + list(data.glob(f'*/{name}')) if p.is_dir()]
    if len(found) != 1:
        sys.exit(f'sequence {name}: expected one folder under {data}, found {len(found)}')
    return found[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True, type=Path,
                    help='folder with the extracted AGHRI archives (dataset_part*/<sequence>/...)')
    ap.add_argument('--aghri-benchmark', required=True, type=Path,
                    help='clone of https://github.com/LCAS/AGHRI-dataset-benchmark')
    ap.add_argument('--split', default='test',
                    help="'val', 'test', a list in splits/ (e.g. 'moving'), "
                         'or a file of sequence names')
    ap.add_argument('--out', required=True, type=Path, help='output folder')
    ap.add_argument('--detections', type=Path, default=None,
                    help='per-frame detection JSONs, <dir>/<sequence>/<pcd stem>.json; '
                         'default: the annotated boxes with identities withheld')
    ap.add_argument('--min-score', type=float, default=0.0,
                    help='drop detections below this score')
    ap.add_argument('--tf-latency', type=float, default=None,
                    help='give the tracker lidar-frame boxes and only the TF up to stamp + this '
                         'many seconds, so it looks the transform up itself (a detector delay)')
    ap.add_argument('--param', action='append', default=[], metavar='NAME:=VALUE',
                    help='tracker parameter, as on the ROS command line (repeatable)')
    ap.add_argument('--publish-default', action='store_true',
                    help="keep the node's vel_filter and height_filter (by default every track is "
                         'published, because AGHRI annotates standing people too)')
    ap.add_argument('--tracker-src', type=Path, default=REPO / 'kf_hungarian_tracker',
                    help='package folder of the tracker to test '
                         '(default: the one in this repository)')
    ap.add_argument('--eval-python', default=sys.executable,
                    help='Python interpreter with motmetrics for the AGHRI evaluator')
    a = ap.parse_args()

    mot3d = a.aghri_benchmark / '3d-tracking' / 'common' / 'mot3d'
    sys.path.insert(0, str(mot3d))
    from bev_iou import iou_matrix
    from gt_as_detections import convert_scene_to_detections, convert_scene_to_gt_mot3d
    from mot3d_writer import load_mot3d

    sys.path.insert(0, str(a.tracker_src))
    import rclpy
    from kf_hungarian_tracker import kf_hungarian_node
    print(f'tracker: {kf_hungarian_node.__file__}')

    names = split_names(a.split, a.aghri_benchmark)
    scenes = [find_scene(a.data, n) for n in names]
    gt_dir, tracks_dir = a.out / 'gt', a.out / 'tracks'
    tracks_dir.mkdir(parents=True, exist_ok=True)

    params = ['global_frame:=map',
              f'transform_to_global_frame:={"true" if a.tf_latency is not None else "false"}']
    if not a.publish_default:
        params += ['vel_filter:=[-1.0,1000000000.0]',
                   'height_filter:=[-1000000000.0,1000000000.0]']
    params += a.param
    rclpy.init(args=['--ros-args'] + [x for p in params for x in ('-p', p)])
    for scene in scenes:
        gt_csv = gt_dir / f'{scene.name}.csv'
        det_dir = a.out / 'gt_detections' / scene.name
        if not gt_csv.exists():
            convert_scene_to_gt_mot3d(scene, gt_csv)
            convert_scene_to_detections(scene, det_dir)
        if a.detections is not None:
            det_dir = a.detections / scene.name
        n, k = run_scene(kf_hungarian_node.KFHungarianTracker, scene, det_dir, a.min_score,
                         a.tf_latency, tracks_dir / f'{scene.name}.csv')
        print(f'{scene.name}: {n} scans, {k} published track ids', flush=True)
    rclpy.shutdown()

    metrics_csv = a.out / 'metrics.csv'
    r = subprocess.run([a.eval_python, str(mot3d / 'evaluate_mot3d.py'), '--gt-dir', str(gt_dir),
                        '--pred-dir', str(tracks_dir), '--out-csv', str(metrics_csv)],
                       capture_output=True, text=True)
    if r.returncode:
        sys.exit(f'AGHRI evaluator failed:\n{r.stdout}{r.stderr}')
    rows = [row for row in csv.DictReader(open(metrics_csv)) if row[''] != 'OVERALL']
    rows = [row for row in rows if row[''] in {s.name for s in scenes}]
    summary = {k: float(np.mean([float(row[k]) for row in rows]))
               for k in ('mota', 'idf1', 'hota')}
    summary.update({k: int(sum(float(row[k]) for row in rows))
                    for k in ('num_switches', 'num_false_positives', 'num_misses')})
    pm = [people_metrics(s, gt_dir / f'{s.name}.csv', tracks_dir / f'{s.name}.csv', load_mot3d,
                         iou_matrix) for s in scenes]
    delays = [d for m in pm for d in m['delays']]
    summary.update(sequences=len(rows), people=len(delays) + sum(m['never'] for m in pm),
                   never_published=sum(m['never'] for m in pm),
                   first_publication_median_s=float(np.median(delays)) if delays else None,
                   spurious_boxes=sum(m['spurious'] for m in pm),
                   dropped_person_frames=sum(m['dropped'] for m in pm),
                   person_frames=sum(m['person_frames'] for m in pm), params=params)
    (a.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f"\n{summary['sequences']} sequences (mean over sequences): "
          f"MOTA {summary['mota']:.3f}  IDF1 {summary['idf1']:.3f}  "
          f"HOTA {summary['hota']:.3f}  |  "
          f"IDSW {summary['num_switches']}  FP {summary['num_false_positives']}  "
          f"FN {summary['num_misses']}")
    med = summary['first_publication_median_s']
    print(f"people: {summary['people']}, first published after "
          f"{'-' if med is None else f'{med:.2f} s'} (median), "
          f"never published {summary['never_published']}  |  "
          f"spurious boxes {summary['spurious_boxes']}  |  "
          f"dropped person-frames {summary['dropped_person_frames']} "
          f"of {summary['person_frames']}")
    print(f'per-sequence metrics: {metrics_csv}')


if __name__ == '__main__':
    main()
