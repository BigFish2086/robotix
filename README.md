# robotix

Cognitive Robotics Project

### Project Modules
- controller.py
- sensors.py
- mapping.py
- slam.py

### Setup
```
git clone https://github.com/BigFish2086/robotix catkin_ws
cd catkin_ws
./init.sh
./config.sh
```

### Start
1. Spin up the simulation on Rviz and Gazebo `./simulation_up.sh`
2. Run each module alone like `rosrun  robotix <module_name>` but you'll need to start Ira-sensor-tools with `./ira_tools_up.sh`
3. OR run them using the launch file which is easier; for example: 
- to test robot mapping with known pose use `roslaunch robotix map.launch`
- and to test robot SLAM (Simultaneous localization and mapping) with unkonwn pose use `roslaunch robotix slam.launch`

##### Note: the `controller.py` won't be running as part of the launch files, it should apart i.e. `rosrun robotix controller.py`
