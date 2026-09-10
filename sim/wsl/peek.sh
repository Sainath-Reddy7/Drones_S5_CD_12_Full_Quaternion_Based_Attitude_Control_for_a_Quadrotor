#!/bin/bash
# peek.sh -- print the newest run CSV's tracking trajectory (1 Hz samples)
cd /root/drones/results
f=$(ls -t steprate_seed*.csv 2>/dev/null | head -1)
[ -z "$f" ] && f=$(ls -t *_seed*.csv | head -1)
echo "FILE=$f"
awk -F, 'NR>2 && (NR-3)%50==0 {printf "t=%5.1f  phi_ref=%+6.2f  phi=%+6.2f  theta=%+6.2f  alpha=%5.1f\n", $1, $11, $14, $15, $2}' "$f" | head -18
