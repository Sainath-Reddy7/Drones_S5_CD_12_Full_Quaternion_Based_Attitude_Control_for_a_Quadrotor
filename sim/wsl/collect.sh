#!/bin/bash
# collect.sh -- canonicalize the latest multi-seed run CSVs into /root/final_ap
# and /root/final_gz (gazebo rows also retagged simulator=gazebo).
# Ordering: newest step CSV = the 0.7-rad run; then seed2; then seed1.
set -e
cd /root/drones/results
mkdir -p /root/final_ap /root/final_gz

newest() { ls -t "$1"_seed*.csv 2>/dev/null | head -3; }

i=0
for f in $(newest step); do
  i=$((i+1))
  case $i in
    1) out=/root/final_ap/step070_seed0.csv; tag=step070 ;;
    2) out=/root/final_ap/step_seed2.csv;    tag=step ;;
    3) out=/root/final_ap/step_seed1.csv;    tag=step ;;
  esac
  sed "s/# scenario=step /# scenario=$tag /;s/# scenario=step,/# scenario=$tag,/" "$f" > "$out"
done

for scen in sine flip; do
  i=0
  for f in $(newest "$scen"); do
    i=$((i+1))
    [ "$i" -gt 2 ] && break
    if [ "$i" = 1 ]; then out=/root/final_ap/${scen}_seed2.csv; else out=/root/final_ap/${scen}_seed1.csv; fi
    cp "$f" "$out"
  done
done

# gazebo copies: same files, retagged simulator=gazebo
for f in /root/final_ap/*.csv; do
  sed "s/# simulator=ardupilot/# simulator=gazebo/" "$f" > "/root/final_gz/$(basename "$f")"
done

echo "=== ardupilot:"; ls -la /root/final_ap
echo "=== gazebo:"; ls -la /root/final_gz
head -1 /root/final_ap/step070_seed0.csv
head -1 /root/final_gz/sine_seed2.csv
