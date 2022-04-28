#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import cv2
import time

def drive():
    velocity_publisher = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
    vel_msg = Twist()

    # Init Twist values
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Get parameters from input
    while not rospy.is_shutdown():
        speed = float(input("Type a speed:"))
        angle = float(input("Type an angle:"))
        isForward = int(input("Is Forward:"))

        vel_msg.angular.z = angle

        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)

        velocity_publisher.publish(vel_msg)
        time.sleep(2)

if __name__ == "__main__":
    rospy.init_node("drive" , anonymous=True)
    try:
        drive()
    except rospy.ROSInterruptException: pass
