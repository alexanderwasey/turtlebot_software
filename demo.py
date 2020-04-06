#!/usr/bin/env python
from __future__ import absolute_import

import rospy

import math 
from driver import Driver

turtle_driver = Driver() #Initialises a driver object 

turtle_driver.forward_by(1.0, 0.4) #Move forward by 1.0m at 0.4m/s 

turtle_driver.turn_by(math.pi/2) #Rotate by 90 degrees clockwise 