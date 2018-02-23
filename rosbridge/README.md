# ROS bridge for AirSim.

1. Catkin is the ROS build system; before you can run the ROS bridge you need to create a symlink where Catkin can see it:
```
ln -s /PATH_TO/airsim_bridge ~/catkin_ws/build
```

2. Start the ROS core:
```
roscore
```

3. Open a second terminal window and start the ROS bridge:
```
rosrun airsim_bridge bridge.py
```

4. If you want to play a rosbag (pre-recorded ROS session), open a third terminal window:
```
rosbag play ROSBAG_FILENAME
```

5. To record a rosbag:
```
rosbag record -a
```
