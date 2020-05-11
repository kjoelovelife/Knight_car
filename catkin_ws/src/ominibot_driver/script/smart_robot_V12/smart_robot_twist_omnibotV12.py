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
import sys, time , select, tty ,termios ,serial
import numpy as np
from smart_robotV12_driver import smart_robotV12

## import ROS library
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

## set param 
veh_cmd = {"Vx":0, "Vy":0, "Omega":0}
vel_gain = rospy.get_param("~vel_gain",0.25)

## Show information of this process

msg = """

Control Smart Robot !
---------------------------
                


                  x
                  ^
 ^                |                ^
 | motor2         |        motor 1 |
                  |
                  |
       y <--------|


            motor3 
             -->


------------------------------
w: Go
s: Stop

CTRL-C to quit
---------------------------

"""


print(msg)
                    
def callback(data):
    global veh_cmd , vel_gain
    twist = data 
    veh_cmd["Vx"] = twist.linear.x * vel_gain
    veh_cmd["Vy"] = twist.linear.y * vel_gain
    veh_cmd["Omega"] = twist.angular.z * vel_gain
    #print(" Vx : {} , Vy : {} , Vz : {}".format(veh_cmd["Vx"],veh_cmd["Vy"] ,veh_cmd["Omega"]))
    tx_vel_cmd()

def tx_vel_cmd():
    cmd = [ int(veh_cmd["Vx"]), int(veh_cmd["Vy"]), int(veh_cmd["Omega"]) ]     
    robot.vel(cmd)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

##  start the process  ##
if __name__ == '__main__':

    ## initialize
    rospy.init_node('smart_robot_twist_omnibotV12', anonymous=True)
    
    ## set serial communication
    port  = "/dev/smart_robot_omnibotV12"     #port = "" for linux
    baud  = 115200
    robot = smart_robotV12(port,baud)
    robot.connect()
    robot.set_mode(0)
    time.sleep(0.5)
    robot.set_mode(0)
    while(True):       
        if rospy.is_shutdown() == True:
            robot.disconnect()
            print''
            break
        else:
            rospy.Subscriber('/cmd_vel', Twist, callback)
            rospy.spin()
    
    
"""
settings = termios.tcgetattr(sys.stdin)
    key = getKey()
    
    if key == '\x03' :
        robot.disconnect()
        break
    elif key == "p":
        print(" You send signal P ")
        robot.send_signal("P")
        time.sleep(0.5) 
        warn_state = False
    else:
        if warn_state == False:
            print( " Please enter s , p , r to send signal to smart robot. " )
            warn_state = True


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
"""
