#! /bin/bash

~/DartTracker/DartTracker 127.0.0.1 0.42 0.06 & PID0=$!
sleep 5s
roslaunch tracker_pkg calc_depth.launch & PID1=$!
sleep 10s

wait
kill -9 PID0 PID1
kill -9 `ps -e | grep DartTracker | awk 'NR==1{print $1}' `
exit