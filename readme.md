## Usage

### Simulation with Gazebo

```
roslaunch simulation mavros_posix_sitl.launch
# wait 10s
# Open QGC
~/Downloads/QGroundControl.AppImage
roslaunch simulation sim_gazebo.launch
```

### Real flight

```
rosrun offboard_pkg all.sh
```