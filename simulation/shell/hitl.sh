#! /bin/bash

# Set FC into HITL mode
# 0. plug the Pixhawk, turn on the remote control
# 1. run jmavsim
#cd ~/Firmware
## ./Tools/jmavsim_run.sh -q -s -d /dev/ttyUSB0 -b 57600 -r 250 & PID0=$!    # wireless
#gazebo Tools/sitl_gazebo/worlds/iris_fpv_cam.worl & PID0=$!     # GPS unlock
#sleep 10s

# 2. start QGroundControl
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" & PID1=$!
sleep 10s

# 3. mavros
roslaunch simulation sim_gazebo.launch & PID2=$!
sleep 10s

## 4. task scripts
#roslaunch simulation sim.launch & PID3=$!

# exit
wait
kill  PID1 PID2
exit
