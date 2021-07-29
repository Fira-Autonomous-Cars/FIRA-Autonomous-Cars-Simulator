
import rospy
import sys
import math
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2


def routine(self):
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        
            try:
                self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                
            except CvBridgeError as e:
                print(e)