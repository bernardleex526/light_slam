#!/bin/bash
# evo 评测：GT vs LIO 轨迹（无 3D 绘图——WSL matplotlib 冲突）
# 用法: ./evo_eval.sh <run_dir>       # run_dir 由 run_eval.sh 生成（含 gt.tum / lio.tum）
set -e
RUN=${1:-}
if [ -z "$RUN" ] || [ ! -d "$RUN" ]; then
  echo "usage: $0 <run_dir>   (dir must contain gt.tum and lio.tum)"
  exit 1
fi
GT=$RUN/gt.tum
TRAJ=$RUN/lio.tum
if [ ! -s "$GT" ] || [ ! -s "$TRAJ" ]; then
  echo "missing files: $GT / $TRAJ"
  exit 1
fi
echo "=== evo_ape: $GT vs $TRAJ ==="
evo_ape tum "$GT" "$TRAJ" -va \
  --save_results $RUN/ape.zip 2>&1 | tail -25
