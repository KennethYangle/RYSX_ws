#! /bin/bash

# Set FC into HITL mode
# 0. plug the Pixhawk, turn on the remote control
# 1. run jmavsim
cd ~/Prometheus/px4Firmware/Firmware_v110
# ./Tools/jmavsim_run.sh -q -s -d /dev/ttyUSB0 -b 57600 -r 250 & PID0=$!    # wireless
./Tools/jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 57600 -r 250 & PID0=$!     # GPS unlock
sleep 10s

# 2. start QGroundControl
cd ~
./Downloads/QGroundControl.AppImage & PID1=$!
sleep 10s

# 3. mavros
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" & PID2=$!
sleep 10s

# 4. task scripts
roslaunch simulation sim.launch & PID3=$!

# exit
wait
kill PID0 PID1 PID2
exit
