# ROS bridge for AirSim.

SETUP
Catkin is the ROS build system. Before you can run the ROS bridge for the first time, you need to create a symlink where Catkin can see it:
```
ln -s /PATH_TO/airsim_bridge ~/catkin_ws/build
```

RUN
1. Start the ROS core:
```
roscore
```

2. Open a second terminal window and start the ROS bridge:
```
rosrun airsim_bridge bridge.py
```

3. If you want to play a rosbag (pre-recorded ROS session), open a third terminal window:
```
rosbag play ROSBAG_FILENAME
```

4. To record a rosbag:
```
rosbag record -a
```
