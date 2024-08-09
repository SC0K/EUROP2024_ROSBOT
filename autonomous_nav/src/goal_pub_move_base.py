#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import time

class MoveBaseSequence:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_base_sequence')

        # Publisher for move_base goals
        self.goal_pub = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped, queue_size=10)

        # List of goals (position and orientation in radians)
        self.goals = [
            {"position": [-2.9, 0.0, 0.0], "orientation": 0.0},
            {"position": [-2.8, 0.0, 0.0], "orientation": 0.0},
            {"position": [-2.6, 0.0, 0.0], "orientation": 0},
            {"position": [-2.5, 0.0, 0.0], "orientation": 0},
            {"position": [-2.3, 0.0, 0.0], "orientation": 0},
        ]

        # Time between sending goals
        self.goal_delay = 0.3  # seconds

    def send_goals(self):
        for i, goal in enumerate(self.goals):
            # Create a PoseStamped message to send as a goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"  # Use "map" or "odom" depending on your setup

            # Set position
            goal_msg.pose.position.x = goal["position"][0]
            goal_msg.pose.position.y = goal["position"][1]
            goal_msg.pose.position.z = goal["position"][2]

            # Set orientation (yaw)
            q = tf.transformations.quaternion_from_euler(0, 0, goal["orientation"])
            goal_msg.pose.orientation.x = q[0]
            goal_msg.pose.orientation.y = q[1]
            goal_msg.pose.orientation.z = q[2]
            goal_msg.pose.orientation.w = q[3]

            # Publish the goal
            rospy.loginfo(f"Sending goal {i+1}: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}, yaw={goal['orientation']}")
            self.goal_pub.publish(goal_msg)

            # Wait for a certain time before sending the next goal
            time.sleep(self.goal_delay)

if __name__ == '__main__':
    try:
        node = MoveBaseSequence()
        node.send_goals()
    except rospy.ROSInterruptException:
        pass
