#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Header
from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.MAX_VELOCITY = 1
        self.relative_x = 0
        self.relative_y = 0
        

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        # notes on coordinate system: 
        relative_angle = math.atan(self.relative_x/self.relative_y)
        if(abs(relative_angle) > 0.2): #slowly turn backwards first if angle off
            if(relative_angle>0 and self.relative_y>0 or relative_angle<0 and self.relative_y<0):
                turn_direction = -1 #turn left backwards
            else:
                turn_direction = 1 #turn right backwards
            velocity = - 0.2
            steering_angle = min(0.5, abs(relative_angle))*turn_direction * 5

        else: #then drive forwards or backwards
            x_diff = self.relative_x - self.parking_distance
            rospy.loginfo("x diff" + str(x_diff))
            move_forward = 1 if (x_diff > 0) else -1
            velocity = min(self.MAX_VELOCITY, abs(x_diff)) * move_forward
            steering_angle = 0
     
        relative_distance = math.sqrt(self.relative_x**2 + self.relative_y**2)
        
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header = Header()
        drive = AckermannDrive()

        drive.steering_angle = steering_angle
        drive.speed = velocity
        rospy.loginfo("speed: " + str(drive.speed))
        drive_cmd.drive = drive
        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
