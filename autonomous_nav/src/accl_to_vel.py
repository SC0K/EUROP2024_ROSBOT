#!/usr/bin/env python3

## This is a prototype
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class AccelToCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('accel_to_cmd_vel')

        # Parameters
        self.accel_x = 0.0
        self.accel_angular = 0.0
        self.vel_x = 0.0
        self.vel_angular = 0.0
        self.last_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/robot1/accel_x', Float32, self.accel_x_callback)
        rospy.Subscriber('/robot1/accel_y', Float32, self.accel_angular_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)

        # Timer to update and publish velocities
        self.timer = rospy.Timer(rospy.Duration(0.5), self.update_and_publish)

    def accel_x_callback(self, msg):
        self.accel_x = msg.data

    def accel_angular_callback(self, msg):
        self.accel_angular = msg.data

    def update_and_publish(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        rospy.loginfo(dt)
        self.last_time = current_time

        # Integrate accelerations to update velocities
        self.vel_x += self.accel_x * dt 
        self.vel_angular += self.accel_angular * dt

        # Create Twist message
        twist = Twist()
        twist.linear.x = self.vel_x
        twist.angular.z = self.vel_angular

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = AccelToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
