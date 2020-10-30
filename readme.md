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
python plot_data.py [-h] [-s] [-p] [-n] [-l LINEWIDTH] [-r RANGE] [-t TIME]
                    log variable [variable ...]
python plot_multiple_file.py [-h] [-s] [-p] [-l LINEWIDTH] [-r RANGE]
                             [-t TIME] [--label LABEL [LABEL ...]]
                             [--title TITLE] [--xlabel XLABEL]
                             [--ylabel YLABEL]
                             log [log ...] variable
# for example
# python plot_data.py ../simulation/log/20200923_104623_sim.log rpos_est_body_raw depth rpos_est_body
# python plot_multiple_file.py ../archive/20mps.log ../archive/10mps.log ../archive/5mps.log pos_i -p -t "30 45" -r "0 640 0 480" --title pos_i --xlabel ex --ylabel ey  --label "20m/s" "10m/s" "5 m/s"
```

### Archive
The log files of our simulation and experiment are stored in the `archive` folder. You can use the analysis tools above to visualize some variables.