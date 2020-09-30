#! /bin/bash

# Set FC into Hex+ mode
roslaunch tracker_pkg calc_depth.launch & PID0=$!
sleep 10s
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600" & PID1=$!
sleep 10s
roslaunch car_pkg px4_ruying.launch & PID2=$!
sleep 10s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 24, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 31, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 32, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s
roslaunch offboard_pkg rysx.launch & PID3=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3
exit