from __future__ import absolute_import
import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Odometer(object):
    
    #Keeps track of the current position and orientation with respect to
    #the initial position and orientation of the turtlebot.
    #Also supports a stopwatch-like functionality for measuring travelled
    #distance or angle.

    #IMPORTANT: Make sure a ROS node has has been initialised before
    #instantiating this class.
    
   
    def __init__(self):
        self.current_pos = (0, 0)
        self.current_ori = 0        
        self._odom_sub = rospy.Subscriber('odom', Odometry, self._odom_callback)

        self._start_pos = (0, 0)
    	self._start_ori = 0

    def _odom_callback(self, data):
        self.current_pos = (data.pose.pose.position.x, data.pose.pose.position.y)
        quaternion = (
                data.pose.pose.orientation.x, 
                data.pose.pose.orientation.y, 
                data.pose.pose.orientation.z, 
                data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.current_ori = euler[2]


    def calc_dist(self, pos1, pos2):
        #Calculate the distance between two 2d points given as tuples.

        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
    

    def reset(self):
        #Starts the stop watch for measuring travelled distance and angle

        self._start_pos = self.current_pos
        self._start_ori = self.current_ori
           
    def get_travelled_dst(self):
        #Returns the absolute value of the linear displacement since the last time reset() was called

        return self.calc_dist(self._start_pos, self.current_pos)

    def get_travelled_angle(self, anti_cw):
        #Returns the value of angular displacement since the last time reset() was called.
        #Returns values in the intervals [0, 2*pi) for clockwise angles and [0, -2*pi) for negative angles.

        #The parameter anti_cw should be set to True if the turn was anti-clockwise and false otherwise.
        
        
        # we have to be careful when the turtlebot passes the orientation of pi,
        # where it changes to -pi.
        # The constant 0.01 was added below to deal with sensor noise. As a 
        # consequence, some return values must be adjusted to not return 
        # small negative values.
        if anti_cw:
            if self._start_ori >= 0:            
                if self.current_ori < 0 or \
                (self.current_ori >= 0 and self.current_ori < self._start_ori - 0.01):
                    return -((self.current_ori + 2*math.pi) - self._start_ori)
                else:
                    return -max(self.current_ori - self._start_ori, 0)
            else: 
                if self.current_ori < 0 and self.current_ori < self._start_ori - 0.01:
                    return -((self.current_ori + 2*math.pi) - self._start_ori)
                else:
                    return -max(self.current_ori - self._start_ori, 0)
        
        else:
            if self._start_ori >= 0: 
                if self.current_ori >= 0 and self.current_ori > self._start_ori + 0.01:
                    return self._start_ori - (self.current_ori - 2*math.pi)
                else:
                    return max(self._start_ori - self.current_ori, 0)
            else:
                if self.current_ori <= self._start_ori + 0.01:
                    return max(self._start_ori - self.current_ori, 0)
                else:
                    return self._start_ori - (self.current_ori - 2*math.pi)