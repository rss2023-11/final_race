#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation_trackfinder import cd_color_segmentation


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
        
        bridge = CvBridge()
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # goal = (int(len(image[0])/2),int(len(image)/2))
        goal = cd_color_segmentation(image)
        if goal!=None: # sanity check 
            center=ConeLocationPixel()
            center.v=goal[1]
            center.u=goal[0]
            
            # image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            non_track_ratio = 0.5
            height=len(image)
            width=len(image[0])
            cv2.rectangle(image, (0,0),(width,int(height*non_track_ratio)), (0,0,0), -1)

            cv2.circle(image, goal, 10, (255, 0, 255), 3)

            bounding_box_img=self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(bounding_box_img)
            
            self.cone_pub.publish(center)
            self.debug_pub.publish(bounding_box_img)
            
            #################################

            #image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        else:
            print("goal is", center)
            bounding_box_img=self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(bounding_box_img)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
