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

<<<<<<< HEAD
	self.speed = rospy.get_param("~speed", 3)

        self.parking_distance = float(rospy.get_param("~parking_distance", 0))
        self.kp = rospy.get_param("~kp", 0.2)
        self.kd = rospy.get_param("~kd", 1)

=======
        self.parking_distance = float(rospy.get_param("~parking_distance", 0)) # meters; try playing with this number!
        self.kp = 0.15
        self.kd = 1
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971
        self.wheelbase_length = 0.35 
        self.prev_error = 0
        self.prev_steering_angle = 0
        self.relative_x = 0
        self.relative_y = 0
        self.am_moving_backwards = True
<<<<<<< HEAD
       
=======
        
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971
    def calculate_steering_angle(self, dx, dy):
        # Calculate the steering angle to the target point using Ackermann steering geometry
        alpha = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)

        steering_angle = math.atan2(2.0 * self.wheelbase_length * math.sin(alpha), distance)
        return steering_angle
    
    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        rospy.logwarn(msg)
<<<<<<< HEAD
        
        relative_angle = math.atan2(self.relative_y, self.relative_x)
        de = relative_angle - self.prev_error
        self.prev_error = relative_angle
        steering_angle = self.calculate_steering_angle(self.relative_x, self.relative_y)
        steering_angle = self.kp * steering_angle + self.kd * de
=======
        # notes on coordinate system: 
        relative_angle = math.atan2(self.relative_y, self.relative_x)
        de = relative_angle-self.prev_error
        self.prev_error = relative_angle
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971

        steering_angle = self.calculate_steering_angle(self.relative_x, self.relative_y) * self.kp + self.kd * de
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header = Header()
        drive = AckermannDrive()
        drive.steering_angle = steering_angle
<<<<<<< HEAD
        drive.speed = self.speed
        drive_cmd.drive = drive

=======
        drive.speed = 2.0
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971
        #rospy.loginfo("speed: " + str(drive.speed))
        #rospy.loginfo("steering angle: " + str(drive.steering_angle))

        rospy.logwarn(drive.steering_angle)
<<<<<<< HEAD

        if abs(drive.steering_angle) < 0.15 and abs(self.prev_steering_angle-steering_angle)<0.026:
            # rospy.logwarn("DRIVE COMMAND")
            # rospy.logwarn(drive_cmd)
            self.prev_steering_angle = steering_angle
=======
        if abs(drive.steering_angle) < 0.15:
>>>>>>> a5d67e42c0618c1ae6f16b29bbf1673ae395c971
            self.drive_pub.publish(drive_cmd)
        # self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        # scaling=(math.sqrt(self.relative_x**2 + self.relative_y**2)-self.parking_distance)/(math.sqrt(self.relative_x**2 + self.relative_y**2))
        scaling = 1

        # error_msg.x_error = self.relative_x*scaling
        # error_msg.y_error = self.relative_y*scaling
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)*scaling
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
