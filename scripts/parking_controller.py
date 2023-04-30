#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Header
from final_race.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    CIRCLE_RADIUS = 0.9 # Radius of circle traced out when turning as fast as possible
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic", "/vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = float(rospy.get_param("~parking_distance")) # meters; try playing with this number!
        self.MAX_VELOCITY = 1
        self.relative_x = 0
        self.relative_y = 0
        self.am_moving_backwards = True
        

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        # notes on coordinate system: 
        relative_angle = math.atan2(self.relative_y, self.relative_x)


        steering_amount = min(0.5, abs(relative_angle))
        steering_angle = steering_amount if self.relative_y > 0 else -steering_amount
       #  rospy.logwarn(steering_amount)

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header = Header()
        drive = AckermannDrive()

        drive.steering_angle = steering_angle
        drive.speed = 1.0
        #rospy.loginfo("speed: " + str(drive.speed))
        #rospy.loginfo("steering angle: " + str(drive.steering_angle))
        drive_cmd.drive = drive
        rospy.logwarn(drive_cmd)
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################
        scaling=(math.sqrt(self.relative_x**2 + self.relative_y**2)-self.parking_distance)/(math.sqrt(self.relative_x**2 + self.relative_y**2))


        error_msg.x_error = self.relative_x*scaling
        error_msg.y_error = self.relative_y*scaling
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)*scaling

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
