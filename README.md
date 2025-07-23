# Turtle Waypoint Navigator

This ROS package moves the turtle in **Turtlesim** through a list of `(x, y)` waypoints using a simple **proportional controller**.

---

## How to Run

```bash
roslaunch turtle_waypoint_navigator waypoint_nav.launch
```

The turtle will automatically navigate to each waypoint and stop when done.

---

## Dependencies

- `rospy`
- `geometry_msgs`
- `turtlesim`
- `std_msgs`
