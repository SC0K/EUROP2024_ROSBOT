#! /usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

def send_ackermann_command():
    rospy.init_node('ackermann_cmd_publisher', anonymous=True)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
    
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        ackermann_cmd = AckermannDrive()
        ackermann_cmd.speed = 1.0
        ackermann_cmd.acceleration = 0.5
        ackermann_cmd.steering_angle = 0.1
        
        pub.publish(ackermann_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_ackermann_command()
    except rospy.ROSInterruptException:
        pass
