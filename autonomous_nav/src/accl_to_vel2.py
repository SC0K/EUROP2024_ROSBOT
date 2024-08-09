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
        rospy.init_node('accel_to_cmd_vel')

        # Parameters
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.current_yaw = 0.0
        self.forward_vel = 0.0
        self.angular_vel = 0.0
        self.max_linear_vel = 1.0  # Maximum linear velocity (m/s)
        self.max_angular_vel = 1.0  # Maximum angular velocity (rad/s)
        self.friction_factor = 0.1  # Simple friction factor
        self.last_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/robot1/accel_x', Float32, self.accel_x_callback)
        rospy.Subscriber('/robot1/accel_y', Float32, self.accel_y_callback)
        rospy.Subscriber('/robot1_tf/robot1_tf/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)

        # Timer to update and publish velocities
        self.timer = rospy.Timer(rospy.Duration(0.3), self.update_and_publish)

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

    def update_and_publish(self, event):
        # Calculate time step since last update
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        rospy.loginfo(f"Time step: {dt} seconds")
        self.last_time = current_time

        # Compute the magnitude and direction of the acceleration vector
        accel_magnitude = math.sqrt(self.accel_x**2 + self.accel_y**2)
        target_angle = math.atan2(self.accel_y, self.accel_x)

        # Compute the difference between the target angle and the current orientation
        angle_diff = target_angle - self.current_yaw

        # Normalize the angle difference to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Compute the forward acceleration and angular velocity
        forward_accel = accel_magnitude * math.cos(angle_diff)
        angular_acceleration = angle_diff

        # Integrate accelerations to update velocities
        self.forward_vel += forward_accel * dt
        self.angular_vel += angular_acceleration * dt

        # Apply friction to gradually reduce velocity (optional, can be commented out if not needed)
        self.forward_vel *= (1 - self.friction_factor * dt)
        self.angular_vel *= (1 - self.friction_factor * dt)

        # Limit velocities to their maximum values
        self.forward_vel = max(min(self.forward_vel, self.max_linear_vel), -self.max_linear_vel)
        self.angular_vel = max(min(self.angular_vel, self.max_angular_vel), -self.max_angular_vel)

        # Create Twist message
        twist = Twist()
        twist.linear.x = self.forward_vel
        twist.angular.z = self.angular_vel

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = AccelToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
