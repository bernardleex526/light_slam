#!/bin/bash
# evo 评测：GT vs LIO 轨迹（无 3D 绘图——WSL matplotlib 冲突）
# 用法: ./evo_eval.sh <scene> <version>
set -e
SCENE=${1:-corridor}
VERSION=${2:-improved}
DATA=/mnt/d/data
mkdir -p $DATA

GT=$DATA/${SCENE}_gt.tum
TRAJ=$DATA/${SCENE}_${VERSION}_traj.tum
if [ ! -s "$GT" ] || [ ! -s "$TRAJ" ]; then
  echo "missing files: $GT / $TRAJ"
  exit 1
fi
echo "=== evo_ape: $GT vs $TRAJ ==="
evo_ape tum "$GT" "$TRAJ" -va \
  --save_results $DATA/${SCENE}_${VERSION}_ape.zip 2>&1 | tail -25
