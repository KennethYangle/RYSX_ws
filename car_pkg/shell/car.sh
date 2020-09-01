#! /bin/bash

roslaunch car_pkg px4_ruying.launch & PID0=$!
sleep 10s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 24, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 31, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s
rosservice call /mavros_ruying/cmd/command '{broadcast: false, command: 511, confirmation: 0, param1: 32, param2: 5000, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}' &
sleep 2s

wait
kill -9 PID0
exit