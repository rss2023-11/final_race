#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visual_servoing.msg import ConeLocation, ConeLocationPixel

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description
PTS_IMAGE_PLANE = [
    [327, 215],
    [320, 194],
    [317, 189],
    [317, 185],
    [558, 202],
    [125, 211],
    [64, 238],
    [557, 227],
    [511, 187],
    [143, 193]
]
<<<<<<< HEAD
=======
######################################################
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left
PTS_GROUND_PLANE = [
    [61, 0],
    [138, 0],
    [215, 0],
    [291, 0],
    [83, -64],
    [83, 51],
    [40, 35],
    [40, -33],
    [209, 124],
    [209, 107],
]
<<<<<<< HEAD
=======
######################################################
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971

METERS_PER_INCH = 0.0254


class HomographyTransformer:
    def __init__(self):
        self.cone_px_sub = rospy.Subscriber("/relative_cone_px", ConeLocationPixel, self.cone_detection_callback)
        self.cone_pub = rospy.Publisher("/relative_cone", ConeLocation, queue_size=10)

        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def cone_detection_callback(self, msg):
        #Extract information from message
        u = msg.u
        v = msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y
<<<<<<< HEAD
        self.cone_pub.publish(relative_xy_msg)
=======
        if x > 0:
            self.cone_pub.publish(relative_xy_msg)
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('homography_transformer')
    homography_transformer = HomographyTransformer()
    rospy.spin()
