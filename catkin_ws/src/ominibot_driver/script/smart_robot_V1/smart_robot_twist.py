#!/usr/bin/env python
# coding=UTF-8

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Developer : Lin Wei-Chih , kjoelovelife@gmail.com 
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


## import library
import sys, time , select, tty ,termios
import numpy as np
from smart_robot_driver import Smart_robot

## import ROS library
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


## Show information of this process

msg = """

Control Smart Robot !
---------------------------
                


                  y
                  ^
                  |               ^
 | motorA         |        motorC |
 v                |
                  |
                  |____________> x


            motorB 
             -->


------------------------------
w: Go
s: Stop

CTRL-C to quit
---------------------------

"""

print(msg)
                    
def callback(data):
    
    twist = data 
    Vx = int(twist.linear.x)
    Vy = int(twist.linear.y)
    Vw = int(twist.angular.z) * -1  
    robot.free_speed( Vx, Vy, Vw)

##  start the process  ##
if __name__ == '__main__':

    ## initialize
    rospy.init_node('smart_robot_twist', anonymous=True)
    
    ## set serial communication
    port  = "/dev/smart_robot"     #port = "" for linux
    baud  = 115200
    robot = Smart_robot(port,baud)
    robot.connect()

    while(True):       
        if rospy.is_shutdown() == True:
            for i in range(3):
                robot.free_speed( 0, 0, 0)
            robot.disconnect()
            print''
            break
        else:
            rospy.Subscriber('/cmd_vel', Twist, callback)
            rospy.spin()
