#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Header
from final_race.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry


class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    CIRCLE_RADIUS = 0.9 # Radius of circle traced out when turning as fast as possible
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic", "/vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar

        # self.cone_sub = rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)
        # self.pose_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.line_follow_callback, queue_size = 1)
        rospy.Subscriber("/relative_cone", ConeLocation, self.line_follow_callback, queue_size = 1)

        #PID controls values
        self.kp = 0.15
        self.kd = 1
        self.prev_error = 0
        self.relative_x = 0
        self.relative_y = 0
        self.am_moving_backwards = True
        
        #pure pursuit values
        self.wheelbase_length = 0.35 
        self.lookahead_dist = 1
        self.max_steering_angle = 0.15
        self.velocity = 3

        # Define variables to store current position and orientation
        self.current_position = None
        self.current_orientation = None

        # Define variable to store the target point
        self.target_position = None

    def line_follow_callback(self, msg):
        """
        """
        #find current position (x, y) and orientation (theta)
        # self.current_position = msg.pose.pose.position
        # quat = msg.pose.pose.orientation #this is given in quaternions
        # self.orientation = math.atan2(2*(quat.z*quat.w + quat.x*quat.y), 1 - 2*(quat.y**2 + quat.z**2))

        # self.current_position = msg.x_pos

        # x = self.current_position.x
        # y = self.current_position.y
        # theta = self.orientation
        x = msg.x_pos
        y = msg.y_pos
        theta = math.atan2(y, x)
        rospy.logwarn("CURRENT X AND Y")
        rospy.logwarn(x)
        rospy.logwarn(y)
        rospy.logwarn("CURRENT ANGLE")
        rospy.logwarn(theta)

        # Compute target point
        tx = x + self.lookahead_dist * math.cos(theta)
        ty = y + self.lookahead_dist * math.sin(theta)
        
        # Compute curvature
        curvature = 2 * ty / (self.lookahead_dist ** 2)
        
        # Compute steering angle
        delta = math.atan(self.wheelbase_length * curvature / self.velocity)

        # Publish control command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header = Header()
        drive_cmd.header.stamp = rospy.Time.now()

        drive = AckermannDrive()
        drive.steering_angle = delta
        drive.speed = drive.velocity

        drive_cmd.drive = drive

        rospy.logwarn("STEERING ANGLE: ")
        rospy.logwarn(drive.steering_angle)
        if abs(drive.steering_angle) < self.max_steering_angle:
            # rospy.logwarn("DRIVE COMMAND")
            # rospy.logwarn(drive_cmd)
            self.drive_pub.publish(drive_cmd)
        
    def calculate_steering_angle(self, dx, dy):
        # Calculate the steering angle to the target point using Ackermann steering geometry
        alpha = math.atan2(dy, dx) #L=dy, R=dx
        distance = math.sqrt(dx**2 + dy**2)

        steering_angle = math.atan2(2.0 * self.wheelbase_length * math.sin(alpha), distance)
        return steering_angle
    
    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        rospy.logwarn(msg)
        # notes on coordinate system: 
        relative_angle = math.atan2(self.relative_y, self.relative_x)
        de = relative_angle-self.prev_error
        self.prev_error = relative_angle
        # steering_angle = self.calculate_steering_angle(self.relative_x, self.relative_y)
        steering_angle = self.calculate_steering_angle(self.relative_x, self.relative_y) * self.kp + self.kd * de
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header = Header()
        drive = AckermannDrive()

        drive.steering_angle = steering_angle
        drive.speed = 4.0
        #rospy.loginfo("speed: " + str(drive.speed))
        #rospy.loginfo("steering angle: " + str(drive.steering_angle))
        drive_cmd.drive = drive
        rospy.logwarn(drive.steering_angle)
        if abs(drive.steering_angle) < 0.15:
            # rospy.logwarn("DRIVE COMMAND")
            # rospy.logwarn(drive_cmd)
            self.drive_pub.publish(drive_cmd)
        # self.error_publisher()
        
    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################
        # scaling=(math.sqrt(self.relative_x**2 + self.relative_y**2)-self.parking_distance)/(math.sqrt(self.relative_x**2 + self.relative_y**2))
        scaling = 1

        # error_msg.x_error = self.relative_x*scaling
        # error_msg.y_error = self.relative_y*scaling
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
