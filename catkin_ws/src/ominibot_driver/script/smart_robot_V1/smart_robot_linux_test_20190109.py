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
from smart_robot import smart_robot
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

                    
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

## setup keyword
move_speed = {
              'w':(0  ,0  ,0  ) ,       # forward
              's':(0  ,0  ,0  ) ,       # stop 
              'x':(0  ,0  ,0  ) ,       # back 
              'a':(0  ,0  ,0  ) ,       # left 
              'd':(0  ,0  ,0  ) ,       # right 
              'q':(0  ,0  ,0  ) ,       # rotation_counterclockwise
              'e':(0  ,0  ,0  ) ,       # rotation_clockwise 
             }


##  start the process  ##
if __name__ == '__main__':

    settings = termios.tcgetattr(sys.stdin)

    ## set serial communication
    port  = "/dev/smart_robot"     #port = "" for linux
    baud  = 115200
    robot = smart_robot(port,baud)
    robot.connect()       
    while(True):
        distance = robot.ultrasonic()
        ## enter the key word        
        key = getKey()
        ## judge what the word enter
        if key == '\x03' :
            robot.disconnect()
            break
        elif key in move_speed.keys() :  
            print ("You enter [{}] ".format(key))   
            if key == 'w':
                robot.go()    
            elif key == 'x':
                robot.back()
            elif key == 's':
                robot.stop()
            elif key == 'a':
                robot.left()
            elif key == 'd':
                robot.right()
            elif key == 'q':
                robot.rotate_counterclockwise()
            elif key == 'e':
                robot.rotate_clockwise()
        elif key == 'u': 
            print ("You enter [{}] ".format(key))   
            print(distance)
              

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
