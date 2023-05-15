#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation, transform_image


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.linepub = rospy.Publisher('/line', Float32MultiArray, queue_size = 10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img = transform_image(image)
        img, lines, found = cd_color_segmentation(img,'')

        if len(lines) == 0:
            #try a second time
            trans = transform_image(img, (10, 130, 70), (50, 210, 120))
            img, lines, found = cd_color_segmentation(trans, '', second_try=True)
        if(len(lines) == []):
            #if second time is still none no lines
            float_array_msg0 = Float32MultiArray()
            rospy.logerr("no lines found on 2 tries :(())")
            float_array_msg0.data = [np.inf, np.inf, np.inf, np.inf]
            self.linepub.publish(float_array_msg0)
        else:
            float_array_msg = Float32MultiArray()
            if(len(lines) > 1):
                rospy.logerr("multiple lines taking last lol")
            lines = lines[-1]
            rospy.logerr("x1{}y1{}x2{}y2".format(lines[0], lines[1], lines[2], lines[3]))
            float_array_msg.data = [lines[0], lines[1], lines[2], lines[3]]
            self.linepub.publish(float_array_msg)
        
        debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
