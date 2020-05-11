#!/usr/bin/env python
# coding=UTF-8

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Developer : Lin Wei-Chih , kjoelovelife@gmail.com , on 2020-02-04
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
import time , struct ,binascii ,math
from serial import Serial , SerialException


class smart_robotV12():
    def __init__(self,port,baud):

        ## setup connected parameter
        self.param = {}
        self.param["device_port"] = port
        self.param["baudrate"]    = baud
        self.imu_decode = {"accel":[0,0,0] , "gyro":[0,0,0]} 
        self.imu_decode = {"accel":[0,0,0] , "gyro":[0,0,0]}
        self.odom_decode = [0,0,0]
        self.odom_seq = 0
        self.cmd_decode = [0 ,0 ,0]
        self.cmd_seq = 0
        self.connected = False
        self.start = False
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

    def connect(self):
        print("Try to connect the Smart Robot")
        try:
            self.device = Serial(self.param["device_port"] , self.param["baudrate"] )
            self.connected = True
            print("Connect done!!")
        except:
            print("Error! Please check smart robot.")

    ## close port
    def disconnect(self):
        if self.connected == True:
            self.device.close() 
            self.connected = False  

    ## Start smartbot and choose " 0 :omnibot" , 1: normal motor without encoder , 2: normal motor with encoder , 3 :Mecanum"
    def set_mode(self, vehicle):
        start_cmd = bytearray(b'\xFF\xFE')
        start_cmd.append(0x80)
        start_cmd.append(0x80)
        start_cmd.append(0x09)
        start_cmd += struct.pack('>h',0 ) # 2-bytes , reserved bit                 
        # 2-bytes , first is mode set ; second is vehicle of robot , 0 for omnibot , 1 for Mecanum  
        start_cmd += struct.pack('>h',vehicle) 
        start_cmd.append(0x00)            # 1-bytes , reserved bit    
	#debug
	print("You set : {} ".format(binascii.hexlify(start_cmd)))
        #print("Please Wait for 5 seconds...")
        #time.sleep(5)
        if self.connected == True:
            self.device.write(start_cmd)
            time.sleep(0.5)

    # send vel_cmd[Vx,Vy,Vz]
    def vel(self, veh_cmd): 
      
        speed = bytearray(b'\xFF\xFE')
        speed.append(0x01)
        speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), 0, 65536 ) ) # 2-bytes , velocity for x axis 
        speed += struct.pack('>h',self.clamp( abs(veh_cmd[0]), 0, 65536 ))  # 2-bytes , velocity for y axis 
        speed += struct.pack('>h',self.clamp( abs(veh_cmd[2]), 0, 65536 ))  # 2-bytes , velocity for z axis 

        # set direction
        direction_x = 0
        direction_y = 0
        direction_z = 0         
 
        if veh_cmd[0] >= 0 :
            direction_y = 0
        else :
            direction_y = math.pow(2,1)
        if veh_cmd[2] >= 0 :
            direction_z = math.pow(2,0)
        else :
            direction_z = 0

        direction = direction_x + direction_y + direction_z
        
        # 1-bytes , direction for x(bit2) ,y(bit1) ,z(bit0) ,and 0 : normal , 1 : reverse
        speed += struct.pack('>b',direction)  
            
        # debug
        print(binascii.hexlify(speed))
        if self.connected == True:       
            self.device.write(speed)
            print("Direction: {}".format(direction))

    # send TT motor vel_cmd
    def TT_motor(self, veh_cmd):
      
        
        speed = bytearray(b'\xFF\xFE')
        speed.append(0x01)
        speed += struct.pack('>h',0) # 2-bytes , reserved bit 
        speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), 0, 65536 ))  # 2-bytes , velocity for y axis 
        speed += struct.pack('>h',self.clamp( abs(veh_cmd[2]), 0, 65536 ))  # 2-bytes , velocity for z axis 

        if veh_cmd[2] >= 0:
            direction_z = 0
        else: 
            direction_z = 1
        if veh_cmd[1] >= 0 :
            direction_y = 0
        else:
            direction_y = 2
       
        direction = direction_y + direction_z
        print("Direction : {}".format(direction))
        
        # 1-bytes , direction for x(bit2) ,y(bit1) ,z(bit0) ,and 0 : normal , 1 : reverse
        speed += struct.pack('>b',direction)  
        # debug
        print(binascii.hexlify(speed))
        if self.connected == True:       
            self.device.write(speed)
            print("Direction: {}".format(direction))

    def TT_motor_function(self, veh_cmd):

        self.param["motor_axis_width"]  = 0.13 #(m)
        self.param["motor_axis_length"] = 0.14 #(m)
        self.param["turn_radius"] = 0.2  # (m)
        # under parameter use turning right
        self.param["V4_turn_radius"] = self.param["turn_radius"] - ( self.param["motor_axis_width"] / 2  )
        self.param["V3_turn_radius"] = self.param["turn_radius"] + ( self.param["motor_axis_width"] / 2  )
        self.param["V2_turn_radius"] = math.pow( math.pow(self.param["V4_turn_radius"],2) + math.pow(self.param["motor_axis_length"],2) ,0.5)
        self.param["V1_turn_radius"] = math.pow( math.pow(self.param["V3_turn_radius"],2) + math.pow(self.param["motor_axis_length"],2) ,0.5)
        self.param["velocity_function_Denominator"] = self.param["V1_turn_radius"] + self.param["V2_turn_radius"] + 2 * self.param["turn_radius"]
        self.param["turn_angle"] = math.atan(self.param["motor_axis_length"] / self.param["turn_radius"])
        max_speed = 10000
        min_speed = 0
        speed = bytearray(b'\xFF\xFE')
        speed.append(0x02)
        # V1 = left_front , V2 = right_front , V3 = left_back , V4 = right_back
        if veh_cmd[1] >= 0 :
            reverse = math.pow(2,3) + math.pow(2,1)
            if veh_cmd[2] == 0:    # go forward and do not turn
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V1
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V2
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V3
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V4
            else:
                if veh_cmd[2] < 0: # go forward and turn right
                    V4 = self.clamp(int(abs( ( self.param["V4_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 4500 )
                    V3 = self.clamp(int(abs( ( self.param["V3_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 8900 )
                    V2 = self.clamp(int(abs( ( self.param["V2_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 6500 )
                    V1 = self.clamp(int(abs( ( self.param["V1_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , max_speed )
                    speed += struct.pack('>h',V1)  # 2-bytes , velocity for V1
                    speed += struct.pack('>h',V2)  # 2-bytes , velocity for V2
                    speed += struct.pack('>h',V3)  # 2-bytes , velocity for V3
                    speed += struct.pack('>h',V4)  # 2-bytes , velocity for V4
                else:              # go back and turn left
                    V4 = self.clamp(int(abs( ( self.param["V3_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 4500 )
                    V3 = self.clamp(int(abs( ( self.param["V4_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 8900 )
                    V2 = self.clamp(int(abs( ( self.param["V1_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 6500 )
                    V1 = self.clamp(int(abs( ( self.param["V2_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , max_speed )
                    speed += struct.pack('>h',self.clamp( V1, min_speed, max_speed ))  # 2-bytes , velocity for V1
                    speed += struct.pack('>h',self.clamp( V2, min_speed, max_speed ))  # 2-bytes , velocity for V2
                    speed += struct.pack('>h',self.clamp( V3, min_speed, max_speed ))  # 2-bytes , velocity for V3
                    speed += struct.pack('>h',self.clamp( V4, min_speed, max_speed ))  # 2-bytes , velocity for V4
                print(" left_front: {} , right_front: {} , left_back: {} , right_back: {}  ".format(V1,V2,V3,V4) )
        else:                     
            reverse = math.pow(2,2) + math.pow(2,0)
            if veh_cmd[2] == 0:    # go back and do not turn 
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V1
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V2
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V3
                speed += struct.pack('>h',self.clamp( abs(veh_cmd[1]), min_speed, max_speed ))  # 2-bytes , velocity for V4
            else:
                if veh_cmd[2] < 0: # go back and turn right
                    V4 = self.clamp(int(abs( ( self.param["V4_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 4500 )
                    V3 = self.clamp(int(abs( ( self.param["V3_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 8900 )
                    V2 = self.clamp(int(abs( ( self.param["V2_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 6500 )
                    V1 = self.clamp(int(abs( ( self.param["V1_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , max_speed )
                    speed += struct.pack('>h',self.clamp( V1, min_speed, max_speed ))  # 2-bytes , velocity for V1
                    speed += struct.pack('>h',self.clamp( V2, min_speed, max_speed ))  # 2-bytes , velocity for V2
                    speed += struct.pack('>h',self.clamp( V3, min_speed, max_speed ))  # 2-bytes , velocity for V3
                    speed += struct.pack('>h',self.clamp( V4, min_speed, max_speed ))  # 2-bytes , velocity for V4
                else:              # go back and turn left
                    V4 = self.clamp(int(abs( ( self.param["V3_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 4500 )
                    V3 = self.clamp(int(abs( ( self.param["V4_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 8900 )
                    V2 = self.clamp(int(abs( ( self.param["V1_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , 6500 )
                    V1 = self.clamp(int(abs( ( self.param["V2_turn_radius"] / self.param["velocity_function_Denominator"] ) * veh_cmd[2] * math.cos(self.param["turn_angle"])  )) + abs(veh_cmd[1]), min_speed , max_speed )
                    speed += struct.pack('>h',self.clamp( V1, min_speed, max_speed ))  # 2-bytes , velocity for V1
                    speed += struct.pack('>h',self.clamp( V2, min_speed, max_speed ))  # 2-bytes , velocity for V2
                    speed += struct.pack('>h',self.clamp( V3, min_speed, max_speed ))  # 2-bytes , velocity for V3
                    speed += struct.pack('>h',self.clamp( V4, min_speed, max_speed ))  # 2-bytes , velocity for V4
                print(" left_front: {} , right_front: {} , left_back: {} , right_back: {}  ".format(V1,V2,V3,V4))
          
        # 1-bytes , direction for x(bit2) ,y(bit1) ,z(bit0) ,and 0 : normal , 1 : reverse
        speed += struct.pack('>b',reverse)  
        # debug
        print(binascii.hexlify(speed))
        if self.connected == True:       
            self.device.write(speed)
            print("Reverse: {}".format(reverse))

    def TT_motor_normal(self, veh_cmd):
        max_speed = 10000
        min_speed = 0
        reverse = 0
        fricition_limit = 400
        speed = bytearray(b'\xFF\xFE')
        speed.append(0x02)
        # V1 = left_front , V2 = right_front , V3 = left_back , V4 = right_back
        if veh_cmd[1] >= 0 :
            self.Vel_car = int(abs(veh_cmd[1] + veh_cmd[2]) + fricition_limit)
            if veh_cmd[2] == 0:
                reverse = math.pow(2,3) + math.pow(2,1)
            elif veh_cmd[2] > 0 :
                reverse = math.pow(2,3) + math.pow(2,2) + math.pow(2,1) + math.pow(2,0)          
            else:
                reverse = 0
        else :
            self.Vel_car = int(abs(veh_cmd[1] + veh_cmd[2]) + fricition_limit)
            reverse = math.pow(2,2) + math.pow(2,0)     
        speed += struct.pack('>h',self.clamp( self.Vel_car, min_speed, max_speed ))  # 2-bytes , velocity for V1
        speed += struct.pack('>h',self.clamp( self.Vel_car, min_speed, max_speed ))  # 2-bytes , velocity for V2
        speed += struct.pack('>h',self.clamp( self.Vel_car, min_speed, max_speed ))  # 2-bytes , velocity for V3
        speed += struct.pack('>h',self.clamp( self.Vel_car, min_speed, max_speed ))  # 2-bytes , velocity for V4         
       
        # 1-bytes , direction for x(bit2) ,y(bit1) ,z(bit0) ,and 0 : normal , 1 : reverse
        speed += struct.pack('>b',reverse)  
        # debug
        print(binascii.hexlify(speed))
        if self.connected == True:       
            self.device.write(speed)
            print("Reverse: {}".format(reverse))

    def set_speed_limit(self , speed_limit):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x01) # Tx[4]
        cmd.append(0x00) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd += struct.pack('>h',self.clamp( abs(speed_limit), 0, 65536 )) # Tx[7] , Tx[8] 
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            time.sleep(0.01)
            self.read_speed_limit()

    def read_speed_limit(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x11) # Tx[4]
        cmd.append(0x00) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x00) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            respond = []
            for index in range(1,7,1):
                respond.append(binascii.hexlify(self.device.read(1)))
            #respond = self.device.readlines()
            print("System mode : {}" .format(respond))


    def load_setting(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x00) # Tx[4]
        cmd.append(0x80) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x01) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            print("OmniboardV12 load setting!!")

    def load_initial(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x00) # Tx[4]
        cmd.append(0x80) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x02) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            print("Initialize OmniboradV12.")

    def write_setting(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x00) # Tx[4]
        cmd.append(0x80) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x03) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            print("Omniboard writing setting and don't do anything , include shutdown the power , it will take 20 seconds ....... ")
            time.sleep(20)
            print("You can restart OmniboardV12 now!")


    #================================================
    # vehicle         : 0 -> omnibot   , 1 -> Mecanum , 2 --> no encoder and no imu , 3 --> no encoder and has imu
    # motor           : 0 -> normal    , 1 -> reverse
    # encode          : 0 -> normal    , 1 -> reverse
    # imu_calibration : 0 -> not ot do , 1 -> do it
    # command         : 0 -> control   , 1 -> APP
    #================================================
    def set_system_mode(self,vehicle=0,motor=0,encoder=0,imu_calibration=1,command=0):      
        # calculate
        value = {}
        value["vehicle"] = vehicle      
        if motor == 1:
            value["motor"] = math.pow(2,4)
        else:
            value["motor"] = 0
        if encoder == 1:
            value["encoder"] = math.pow(2,5)
        else:
            value["encoder"] = 0
        if imu_calibration == 1:
            value["imu_calibration"] = math.pow(2,6)
        else:
            value["imu_calibration"] = 0
        if command == 1:
            value["command"] = math.pow(2,7)
        else:
            value["command"] = 0

        mode = value["vehicle"] + value["motor"] + value["encoder"] +  value["imu_calibration"] + value["command"]
        print("Mode : {}".format(mode))
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x09) # Tx[4]
        cmd.append(0x00) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd += struct.pack('>h',mode) # Tx[7] ,Tx[8]
        print("Omniboard write setting!")
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            print("Send to omniboardV12 : {} ".format(binascii.hexlify(cmd)))

    def read_system_mode(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x19) # Tx[4]
        cmd.append(0x00) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x00) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            respond = []
            for index in range(1,7,1):
                respond.append(binascii.hexlify(self.device.read(1)))
            #respond = self.device.readlines()
            print("System mode : {}" .format(respond))

    def read_imu(self):
        cmd = bytearray(b'\xFF\xFE') # Tx[0] , Tx[1]
        cmd.append(0x80) # Tx[2]
        cmd.append(0x80) # Tx[3]
        cmd.append(0x3F) # Tx[4]
        cmd.append(0x00) # Tx[5]
        cmd.append(0x00) # Tx[6]
        cmd.append(0x00) # Tx[7]
        cmd.append(0x00) # Tx[8]
        cmd.append(0x00) # Tx[9]
        if self.connected == True:       
            self.device.write(cmd)
            respond = []
            for index in range(1,25,1):
                respond.append(binascii.hexlify(self.device.read(1)))
            #respond = self.device.readlines()
            print("System mode : {}" .format(respond))


     




