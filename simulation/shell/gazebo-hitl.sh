#! /bin/bash
# 2. start QGroundControl
cd ~/QGC
./QGroundControl.AppImage & PID1=$!
sleep 10s

# 4. task scripts
roslaunch simulation sim_gazebo.launch & PID2=$!

# exit
wait
kill PID1 PID2
exit
