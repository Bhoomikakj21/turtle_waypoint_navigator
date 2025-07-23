#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

waypoints = [(2, 2), (8, 2), (8, 8), (2, 8)]
current_pose = None
goal_index = 0
tolerance = 0.1

def pose_callback(msg):
    global current_pose
    current_pose = msg

def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def control_loop():
    global goal_index

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown() and goal_index < len(waypoints):
        if current_pose is None:
            rate.sleep()
            continue

        goal_x, goal_y = waypoints[goal_index]
        dx = goal_x - current_pose.x
        dy = goal_y - current_pose.y
        dist = distance(current_pose.x, current_pose.y, goal_x, goal_y)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - current_pose.theta

        # Normalize angle between -pi and pi
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()

        # Proportional controller
        if abs(angle_diff) > 0.1:
            twist.angular.z = 1.5 * angle_diff
        else:
            twist.linear.x = 2.0 * dist
            twist.angular.z = 1.5 * angle_diff

        if dist < tolerance:
            rospy.loginfo(f"Reached waypoint {goal_index + 1}")
            goal_index += 1
            twist.linear.x = 0
            twist.angular.z = 0

        pub.publish(twist)
        rate.sleep()

    rospy.loginfo("✅ All waypoints reached.")
    pub.publish(Twist())  # Stop the turtle

if __name__ == '__main__':
    rospy.init_node('waypoint_navigator')
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    control_loop()
