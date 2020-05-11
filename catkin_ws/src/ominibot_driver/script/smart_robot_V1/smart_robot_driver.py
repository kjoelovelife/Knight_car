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
import numpy as np
from serial import Serial

class Smart_robot():
    def __init__(self,port,baud):
    
        ## setup connect parameter
        self.port = port
        self.baud = baud
        self.connected = False

        ## setup parameter
        ## initial
        ## ( vx , vy , w ) , and speed range is 0-255 , if value=127 is stop. 
        self.stop_speed = 127
        self.vx = self.stop_speed  
        self.vy = self.stop_speed
        self.w  = self.stop_speed 
        self.angle = 360 ## angle of ominibot 
        self.length = 12 ## distance between center of ominibot and center of wheels 
        self.linear_speed = np.matrix( [ [self.vx] , [self.vy] , [self.w] ] ) 
        self.radian = np.pi/180
        
        ## setup speed parameter
        self.speed_parameter_1 = np.array([ -1 * np.cos(60 * self.radian) , -1 * np.sin(60 * self.radian) , self.length ])
        self.speed_parameter_2 = np.array([ 1 , 0 , self.length ])
        self.speed_parameter_3 = np.array([ -1 * np.cos(60 * self.radian) , np.sin(60 * self.radian) , self.length ])
        self.speed_parameter = np.matrix( [ self.speed_parameter_1 , self.speed_parameter_2 , self.speed_parameter_3 ] )

        ## speed formula
        self.motor_speed = np.dot(self.speed_parameter ,self.linear_speed )

        ## set motor speed
        self.motor_A = int ( round(self.motor_speed[0] , 0 ) )
        self.motor_B = int ( round(self.motor_speed[1] , 0 ) )
        self.motor_C = int ( round(self.motor_speed[2] , 0 ) )

        ## set send signal
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"

        ## default move_speed[ key word ][ (vx) ,(vy) ,(w) ]
        self.move_speed = {
                       'w':(0  , 50,  0  ) ,       # forward
                       's':(0  ,  0,  0  ) ,       # stop 
                       'x':(0  ,-50,  0  ) ,       # back 
                       'a':(-50,  0,  0  ) ,       # left 
                       'd':(50 ,  0,  0  ) ,       # right 
                       'q':(0  ,  0,  50 ) ,       # rotation_counterclockwise
                       'e':(0  ,  0, -50 ) ,       # rotation_clockwise 
                      }

        ## setup judge_sendSignal
        self.judge_sendSignal = self.send_signal

    ## connect port
    def connect(self):
        try:
            self.microcontroller = Serial(self.port , self.baud)
            self.connected = True
        except SerialException:
            print(" Seems like you dont connect the microcontroller with USB. Please checkit and restart this program. ")
            self.microcontroller.close
    
    ## Ultrasonic 
    def ultrasonic(self):
        self.distance = self.microcontroller.readline() 
        return self.distance        

    ## set speed    
    def set_speed(self,vx_speed,vy_speed,w_speed):
        self.move_speed = {
                           'w':(0  , vy_speed     ,  0  ) ,       # forward
                           's':(0  , 0,  0              ) ,       # stop 
                           'x':(0  , -1 * vy_speed,  0  ) ,       # back 
                           'a':(-1 * vx_speed,  0 ,  0  ) ,       # left 
                           'd':(vx_speed     ,  0 ,  0  ) ,       # right 
                           'q':(0  ,  0,  w_speed       ) ,       # rotation_counterclockwise
                           'e':(0  ,  0, -1 * w_speed   ) ,       # rotation_clockwise 
                          }
    
    ## free action
    def free_speed(self , vx_speed , vy_speed , w_speed):
        self.vx = vx_speed + self.stop_speed
        self.vy = vy_speed + self.stop_speed
        self.w  = w_speed  + self.stop_speed               
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!" 

        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will free moving. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )
            self.vx = 0
            self.vy = 0
            self.w  = 0


    ## forward  [speed]Y => speed > 127  
    def go(self): 
        self.vx = self.move_speed['w'][0] + self.stop_speed
        self.vy = self.move_speed['w'][1] + self.stop_speed   
        self.w  = self.move_speed['w'][2] + self.stop_speed                
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will go foward. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## back     [speed]Y => speed < 127
    def back(self): 
        self.vx = self.move_speed['x'][0] + self.stop_speed
        self.vy = self.move_speed['x'][1] + self.stop_speed   
        self.w  = self.move_speed['x'][2] + self.stop_speed                
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will back. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## stop    
    def stop(self): 
        self.vx = self.move_speed['s'][0] + self.stop_speed
        self.vy = self.move_speed['s'][1] + self.stop_speed   
        self.w  = self.move_speed['s'][2] + self.stop_speed                 
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will stop. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## left    [speed]X => speed < 127   
    def left(self): 
        self.vx = self.move_speed['a'][0] + self.stop_speed
        self.vy = self.move_speed['a'][1] + self.stop_speed   
        self.w  = self.move_speed['a'][2] + self.stop_speed                
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will turn left. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## right   [speed]X => speed > 127 
    def right(self): 
        self.vx = self.move_speed['d'][0] + self.stop_speed
        self.vy = self.move_speed['d'][1] + self.stop_speed   
        self.w  = self.move_speed['d'][2] + self.stop_speed             
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will turn right. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## rotate clockwise [speed]A => speed > 127    
    def rotate_clockwise(self): 
        self.vx = self.move_speed['e'][0] + self.stop_speed
        self.vy = self.move_speed['e'][1] + self.stop_speed   
        self.w  = self.move_speed['e'][2] + self.stop_speed                
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will rotate in the counterclockwise. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )

    ## rotate counterclockwise  [speed]A => speed < 127    
    def rotate_counterclockwise(self): 
        self.vx = self.move_speed['q'][0] + self.stop_speed
        self.vy = self.move_speed['q'][1] + self.stop_speed   
        self.w  = self.move_speed['q'][2] + self.stop_speed                
        self.send_signal = "$AP0:" + str(self.vx) + "X" + str(self.vy) + "Y" + str(self.w) + "A" + "360B!"  
        if self.judge_sendSignal != self.send_signal:       
            print("  smart robot will rotate in the clockwise. \nsend: {} ".format(self.send_signal) )
            self.judge_sendSignal = self.send_signal
        if self.connected == True:
            self.microcontroller.write( bytes( self.send_signal) )
    ## close port
    def disconnect(self):
        if self.connected == True:
            self.microcontroller.close
            self.connected = False
