#!/bin/bash
# perf_sample.sh — 按进程名模式采样 CPU%（/proc stat 差分）与 RSS，输出 CSV
# 用法: ./perf_sample.sh <进程名模式> <时长秒> <输出csv>
# 采样间隔 1s；CPU% = (utime+stime 差 / HZ) / 间隔 * 100
set -u
PATTERN=${1:-run_slam_online}
DUR=${2:-90}
OUT=${3:-/tmp/perf.csv}
HZ=$(getconf CLK_TCK)
echo "ts,pid,cpu_pct,rss_kb" > "$OUT"
START=$(date +%s)
declare -A PREV_CPU PREV_TS
while [ $(( $(date +%s) - START )) -lt "$DUR" ]; do
  for pid in $(pgrep -f "$PATTERN" 2>/dev/null); do
    # 跳过 shell 包装进程（bash -c）自身
    comm=$(cat /proc/$pid/comm 2>/dev/null)
    [ "$comm" = "bash" ] && continue
    [ "$comm" = "sh" ] && continue
    utime=$(awk '{print $14}' /proc/$pid/stat 2>/dev/null)
    stime=$(awk '{print $15}' /proc/$pid/stat 2>/dev/null)
    rss=$(awk '/VmRSS/{print $2}' /proc/$pid/status 2>/dev/null)
    [ -z "$utime" ] && continue
    now=$(date +%s%N)
    cpu=""
    if [ -n "${PREV_CPU[$pid]:-}" ]; then
      dt=$(( (now - ${PREV_TS[$pid]}) / 1000000 ))  # ms
      dcpu=$(( utime + stime - PREV_CPU[$pid] ))
      if [ "$dt" -gt 0 ]; then
        # CPU% = Δticks / HZ / Δt秒 * 100 = d * 100000 / (h * t)，t 单位为 ms
        cpu=$(awk -v d="$dcpu" -v t="$dt" -v h="$HZ" 'BEGIN{printf "%.1f", d*100000/(h*t)}')
      fi
    fi
    PREV_CPU[$pid]=$(( utime + stime ))
    PREV_TS[$pid]=$now
    echo "$(date +%s),$pid,${cpu:-0},${rss:-0}" >> "$OUT"
  done
  sleep 1
done
echo "perf_sample done: $OUT"
