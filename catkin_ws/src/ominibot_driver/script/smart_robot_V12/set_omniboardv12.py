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

##  start the process  ##
if __name__ == '__main__':


    ## set serial communication
    port  = "/dev/smart_robot_omnibotV12"     #port = "" for linux
    baud  = 115200
    robot = smart_robotV12(port,baud)
    robot.connect()
    robot.read_system_mode()
    

    ## set parameter
    mode = {'vehicle':0 , 'motor':0 , 'encoder':0 , 'imu':0 , 'command':0 }  

    ## set vehicle
    print("")
    print("Set kind of vehicle. 0 :omnibot , 1: normal motor without encoder , 2: normal motor with encoder , 3 :Mecanum .") 
    mode["vehicle"] = int(input("Kind of vehicle : "))

    ## set motor
    print("")
    print("Set mode of vehicle. 0 for normal , 1 for reverse .") 
    mode["motor"] = int(input("mode of motor : "))

    ## set encoder
    print("")
    print("Set mode of encoder. 0 for normal , 1 for reverse .") 
    mode["encoder"] = int(input("mode of encoder : "))

    ## set imu_calibration
    print("")
    print("Do you wnat to start imu calibrate when start Omniboard?  0 for don't calibrate , 1 for calibrate .")
    mode["imu"] = int(input("mode of imu calibration : "))

    ## set mode of command
    print("")
    print("set mode of command.   0 for control mode , 1 for APP mode .")
    mode["command"] = int(input("mode of command : "))

    ## set speed limit
    print("")
    print("set speed limit : 60 ~ 65536 .")
    speed_limit = int(input("speed limit : "))
  
    ## send serial

    for number in range(3):
        robot.set_system_mode(vehicle=mode["vehicle"],motor=mode["motor"],encoder=mode["encoder"],imu_calibration=mode["imu"],command=mode["command"]) #    robot.set_system_mode(vehicle,motor,encoder,imu_calibration,command)
    robot.set_speed_limit(speed_limit)
    robot.write_setting()
    robot.disconnect()

