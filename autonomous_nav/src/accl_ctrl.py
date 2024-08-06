#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random

def publish_accelerations():
    rospy.init_node('acceleration_publisher', anonymous=True)
    
    accel_x_pub = rospy.Publisher('/robot1/accel_x', Float32, queue_size=10)
    accel_y_pub = rospy.Publisher('/robot1/accel_y', Float32, queue_size=10)
    
    rate = rospy.Rate(2)  # Hz

    while not rospy.is_shutdown():
        # Generate random acceleration values for demonstration
        accel_x = random.uniform(-1.0, 1.0)
        accel_angular = random.uniform(-1.0, 1.0)
        
        rospy.loginfo(f"Publishing accelerations: x={accel_x}, y={accel_angular}")
        
        accel_x_pub.publish(accel_x)
        accel_y_pub.publish(accel_angular)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_accelerations()
    except rospy.ROSInterruptException:
        pass
