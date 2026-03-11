# START UP GUIDE
This guides how to use the system on lite3 to interact with server and drive the quadruped to move

## Terminal 1
Launch sensors, SLAM and other required nodes 
```bash
cd /home/jetson_avs/robotdog_lite3_ws/
 ./node_manager.sh 
```


## Terminal 2
Launch Nav2 framework for Lite3
```bash
cd /home/jetson_avs/ws_test/lite_nav2_bt/
source install/setup.bash
ros2 launch lite3_nav2_bringup lite3_nav2_navigation.launch.py
```

## Terminal 3
Open RViz for visualization
```bash
rviz2
```

## Terminal 4
Launch node to send request to server. The waypoints in the result will be published on the `/waypoints/raw` topic. The interpolated path will be visualized in RViz. 

```bash
source install/setup.bash
ros2 launch communicator_pkg model_communicator.launch.py
```

## Terminal 5
Run following command to trigger the behavior tree
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```


