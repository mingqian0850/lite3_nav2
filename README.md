# lite3_nav2

## Utilities

### Generate FollowPath goal with orientations aligned to path direction

This package provides a small helper to generate a `nav2_msgs/action/FollowPath` goal YAML
where each pose orientation is automatically set to face the next waypoint direction.

Example:

```bash
ros2 run lite3_nav2_bringup follow_path_goal_gen --frame odom --points "2,-2;5,-4;6,0"
```
