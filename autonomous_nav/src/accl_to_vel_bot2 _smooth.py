#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
import tf.transformations

class AccelToCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('accel_to_cmd_vel2')

        # Parameters
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.current_yaw = 0.0
        self.forward_vel = 0.0
        self.angular_vel = 0.0
        self.max_linear_vel = 0.1 # Maximum linear velocity (m/s)
        self.max_angular_vel = 6  # Maximum angular velocity (rad/s)
        self.last_time = rospy.Time.now()

        # List to store previous angular velocities for smoothing
        self.previous_angular_velocities = [0.0, 0.0]  # Initialize with two zeros
        
        # Weight coefficients for the smoothing (weights must sum to 1 for a proper weighted average)
        self.weights = [0.05, 0.1, 0.85] 

        # Subscribers
        rospy.Subscriber('/robot2/accel_x', Float32, self.accel_x_callback)
        rospy.Subscriber('/robot2/accel_y', Float32, self.accel_y_callback)
        rospy.Subscriber('/robot2/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)

        # Timer to update and publish velocities
        self.timer = rospy.Timer(rospy.Duration(0.2), self.update_and_publish)

    def accel_x_callback(self, msg):
        self.accel_x = msg.data

    def accel_y_callback(self, msg):
        self.accel_y = msg.data

    def odom_callback(self, msg):
        # Extract the current yaw orientation from the odometry message
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.forward_vel = msg.twist.twist.linear.x
        
        # Update angular velocity with the new message
        self.angular_vel = msg.twist.twist.angular.z

    def update_and_publish(self, event):
        # Calculate time step since last update
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Compute the forward acceleration and angular velocity
        if self.forward_vel == 0:
            if math.sin(self.current_yaw) == 0:
                forward_accel = self.accel_x / math.cos(self.current_yaw)
            elif math.cos(self.current_yaw) == 0:
                forward_accel = self.accel_y / math.sin(self.current_yaw)
            else:
                forward_accel = self.accel_x / math.cos(self.current_yaw)
        else:
            if math.sin(self.current_yaw) == 0:
                forward_accel = self.accel_x / math.cos(self.current_yaw)
                self.angular_vel = self.accel_y / self.forward_vel / math.cos(self.current_yaw)
            elif math.cos(self.current_yaw) == 0:
                forward_accel = self.accel_y / math.sin(self.current_yaw)
                self.angular_vel = -self.accel_x / self.forward_vel / math.sin(self.current_yaw)
            else:
                self.angular_vel = (self.accel_y / math.cos(self.current_yaw) - self.accel_x * math.tan(self.current_yaw) / math.cos(self.current_yaw)) / self.forward_vel / ((math.tan(self.current_yaw)) ** 2 + 1)
                forward_accel = (self.accel_y - self.forward_vel * math.cos(self.current_yaw) * self.angular_vel) / math.sin(self.current_yaw)

        # Integrate accelerations to update velocities
        self.forward_vel += forward_accel * dt

        # Smooth the angular velocity using the weighted sum of the previous two and the current angular velocity
        smoothed_angular_vel = (
            self.weights[0] * self.previous_angular_velocities[0] +
            self.weights[1] * self.previous_angular_velocities[1] +
            self.weights[2] * self.angular_vel
        )

        # Update the list of previous angular velocities
        self.previous_angular_velocities = [self.previous_angular_velocities[1], self.angular_vel]

        # Limit forward velocity to prevent excessive speed
        self.forward_vel = max(min(self.forward_vel, self.max_linear_vel), -self.max_linear_vel)
        rospy.loginfo(f"Forward velocity robot2: {self.forward_vel:.2f} m/s")

        # Limit angular velocity to prevent excessive turning
        smoothed_angular_vel = max(min(smoothed_angular_vel, self.max_angular_vel), -self.max_angular_vel)

        # Create Twist message
        twist = Twist()
        twist.linear.x = self.forward_vel
        twist.angular.z = smoothed_angular_vel

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = AccelToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
