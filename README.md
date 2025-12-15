# lite3_nav2

## Utilities

### Generate FollowPath goal with orientations aligned to path direction

This package provides a small helper to generate a `nav2_msgs/action/FollowPath` goal YAML
where each pose orientation is automatically set to face the next waypoint direction.

Example:

```bash
ros2 run lite3_nav2_bringup follow_path_goal_gen --frame odom --points "2,-2;5,-4;6,0"
```

### Execute sequential /follow_path segments from YAML

`follow_path_sequence_executor` reads a YAML file describing ordered waypoints,
automatically computes the heading between each pair, and sequentially sends
`nav2_msgs/action/FollowPath` goals (one segment at a time):

```yaml
# example_path.yaml
frame_id: odom
points:
  - {x: 0.0, y: 0.0}
  - {x: 2.0, y: -2.0}
  - {x: 5.0, y: -4.0}
  - {x: 6.0, y: 0.0}
```

```bash
ros2 run lite3_nav2_bringup follow_path_sequence_executor --path-file example_path.yaml
# 或者使用内置示例：
# ros2 run lite3_nav2_bringup follow_path_sequence_executor --path-file $(ros2 pkg prefix lite3_nav2_bringup)/share/lite3_nav2_bringup/params/waypoints.yaml
```
