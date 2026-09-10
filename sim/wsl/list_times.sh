#!/bin/bash
# list_times.sh -- all run CSVs with mtimes, to split ap vs gz queue windows
cd /root/drones/results
ls -l --time-style=+%H:%M:%S *seed*.csv | awk '{print $6, $7}' | sort
