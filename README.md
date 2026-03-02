# lite3_nav2

## Waypoints pipeline

This stack expects a Path source and then follows it via BT:

- `waypoints_publisher` loads `params/waypoints.yaml`, computes headings, and publishes `/waypoints/raw`.
- `PathFromTopic` (BT node) subscribes `/waypoints/raw`, interpolates by `step`, and feeds `FollowPath`.
- `visualize_waypoints` subscribes `/waypoints/raw` and publishes interpolated markers for RViz2.

Example:

```bash
ros2 run lite3_nav2_bringup waypoints_publisher --topic /waypoints/raw
ros2 run lite3_nav2_bringup visualize_waypoints --waypoints-topic /waypoints/raw --step 0.05
```
