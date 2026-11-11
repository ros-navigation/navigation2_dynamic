#!/bin/bash
# run_bench.sh <tag> <profile> <domain> <tracker_params.yaml> [extra env e.g. B_PMISS=0.3]
TAG=$1; PROFILE=$2; DOM=$3; PARAMS=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd); OUT=$HERE/out; mkdir -p $OUT
# source your ROS 2 installation and the workspace that contains kf_hungarian_tracker first
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 B_PROFILE=$PROFILE B_DURATION=${B_DURATION:-60}
for kv in "$@"; do export "$kv"; done
TRK=$(ros2 pkg prefix kf_hungarian_tracker)/lib/kf_hungarian_tracker/kf_hungarian_node
python3 $TRK --ros-args --params-file $PARAMS > $OUT/$TAG.trk.log 2>&1 & T=$!
python3 $HERE/eval_node.py $OUT/$TAG.json $B_DURATION > $OUT/$TAG.eval.log 2>&1 & E=$!
sleep 2
python3 $HERE/scenario_node.py > $OUT/$TAG.scn.log 2>&1
wait $E; kill -INT $T 2>/dev/null; sleep 1; kill -9 $T 2>/dev/null
echo "$TAG $(cat $OUT/$TAG.json 2>/dev/null | tr -d '\n ')"
