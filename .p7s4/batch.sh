#!/bin/bash
# 批量跑 CP-P7-B：参数 = 一串 "场景:层:时长" 三元组
set -u
for spec in "$@"; do
  IFS=: read -r SCEN LAYER DUR <<< "$spec"
  OUT="/workspace/.p7s4/${SCEN}_${LAYER}_$(date +%H%M%S).csv"
  echo ""
  echo "############ $SCEN / $LAYER ############"
  bash /workspace/.p7s4/p7s4_run.sh "$SCEN" "$LAYER" "$DUR" "$OUT"
  sleep 5
done
echo "############ 批量结束 ############"
