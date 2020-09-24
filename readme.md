## Usage

### Simulation with Gazebo
In `settings.json`, "MODE": "Simulation".
```
roslaunch simulation mavros_posix_sitl.launch
# another terminal
roscd simulation/shell/
./gazebo-hitl.sh | tee -a `roscd simulation/log/ && pwd`/`date +%Y%m%d_%H%M%S_sim.log`
```

### Real flight
In `settings.json`, "MODE": "RealFlight".
```
roscd offboard_pkg/shell/
./all.sh | tee -a `roscd simulation/log/ && pwd`/`date +%Y%m%d_%H%M%S_fly.log`
```

### Analyse log
```
cd PATH_TO_analyse
python PATH_TO_LOG variables ...
# for example
# python plot_data.py ../simulation/log/20200923_104623_sim.log rpos_est_body_raw depth rpos_est_body
```