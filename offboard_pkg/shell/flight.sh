#! /bin/bash

# Set FC into Hex+ mode
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"
