# ros2_rt_dualarm
## build
```bash
    cd ~/ros2_foxy
    colcon build --symlink-install --packages-select ros2_rt_dualarm
```
## start
```bash
    cd ~/ros2_foxy
    ros2 launch ros2_rt_dualarm dualarm.launch.py
```

## simulation start
```bash
    cd ~/ros2_foxy
    ros2 launch ros2_rt_dualarm dualarm_sim.launch.py
```
