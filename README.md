# lite3_nav2

For more info please refer START_UP.md

## Waypoints pipeline

A command in terminal to trigger the behaviour tree
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: odom}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback

```
