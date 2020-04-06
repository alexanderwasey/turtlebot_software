from __future__ import absolute_import
import math
import threading
from time import sleep

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import collisions
from .exceptions import MovementObstructed
from .odometry import Odometer

from time import time


class Driver(object):

    #Initialises a ROS node, publishers and subscribers on instantiation. 
    #Has methods for controlling the robot's wheels.

    curr_speed = 0.0
    
    def __init__(self):
        rospy.init_node(u'findo_driver', anonymous=True)
        rospy.on_shutdown(self.end_driving)

        self.odometer = Odometer()
  
        self._stop_event = threading.Event()
        
        self.cmd_vel_pub = rospy.Publisher(u'cmd_vel', Twist, queue_size=10)
        sleep(1) # some time is necessary for the Publisher to register with the topic

    def rotate_speed(self, initang, currang): #Used to calculate the correction needed to continue straight
        diff = currang - initang
        return -1 * diff

    def dist_to_stop(self, speed): 
        return (speed * speed * 0.5)

    def move_motor(self, fwd, ang):

      #Issues a move command to the motor, with specified linear and angular velocity.
      #The motor maintains the velocities until commanded otherwise.
      
      mc = Twist()
      mc.linear.x = -fwd
      mc.angular.z = ang
      self.cmd_vel_pub.publish(mc)

    def forward_by(self, dst, speed=0.2):
        #Drive the robot by the specified distance (m) at the specified speed (m/s).
        #Use negative distance to drive backward; do not use a negative speed.
        
        init_ang = self.odometer.current_ori
        self.odometer.reset()
        if dst < 0:
            speed = -speed
            clear_ahead = collisions.is_clear_behind
        else:
            clear_ahead = collisions.is_clear_in_front

        self.accelerate(speed, init_ang) #Accelerate to the speed we need
        while(self.odometer.get_travelled_dst() < (abs(dst) - self.dist_to_stop(speed))): #Account for the fact that have stopping distance now
            if clear_ahead():
                try:
                    curr_ang = self.odometer.current_ori
                    rotate = self.rotate_speed(init_ang, curr_ang)
                    self.move_motor(speed, rotate)
                    sleep(0.001)  # necessary to let the up-to-date odometry data to come through
                except rospy.ROSInterruptException:
                    pass
            else:
                self.stop()
                raise MovementObstructed()
        else:
            self.decelerate()

    def turn_by(self, angle, speed=0.2):
        #Turn the robot by the specified angle in radians at
        #the specified angular speed in rad/s. Use a negative angle
        #to turn anticlockwise; do not use a negative speed.
        
        self.odometer.reset()
        if angle < 0:
            anti_cw = True
            clear_ahead = collisions.is_left_turn_clear
        else:
            speed = -speed
            anti_cw = False               
            clear_ahead = collisions.is_right_turn_clear
        while abs(self.odometer.get_travelled_angle(anti_cw)) < abs(angle):
            if clear_ahead():
                self.move_motor(0, speed)
            else:
                self.stop()
                raise MovementObstructed()
            sleep(0.001)  # necessary to let the up-to-date odometry data to come through
        else:
            self.move_motor(0, 0)

    def accelerate(self, speed, ang): #Stability issues if try and accelerate immediately
        if (speed < 0): 
           while (self.curr_speed > speed): 
                curr_ang = self.odometer.current_ori
                rotate = self.rotate_speed(ang, curr_ang)
                self.move_motor(self.curr_speed, rotate)
                sleep(0.01)
                self.curr_speed -= 0.005
        
        else: #Speed > 0
            while(self.curr_speed < speed): 
                curr_ang = self.odometer.current_ori
                rotate = self.rotate_speed(ang, curr_ang)
                self.move_motor(self.curr_speed, rotate)
                sleep(0.01)
                self.curr_speed += 0.005

    def decelerate(self):   
        if (self.curr_speed < 0): 
            while (self.curr_speed < 0): 
                self.move_motor(self.curr_speed, 0)
                self.curr_speed += 0.005
                sleep(0.01)
        else: 
            while(self.curr_speed > 0): 
                self.move_motor(self.curr_speed, 0)
                self.curr_speed -= 0.005
                sleep(0.01)
        self.move_motor(0,0)