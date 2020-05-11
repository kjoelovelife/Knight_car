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
import uuid
from smart_robot import smart_robot
from bluetooth import *

server_socket=BluetoothSocket(RFCOMM)
server_socket.bind(("", PORT_ANY))
server_socket.listen(1)
port = server_socket.getsockname()[1]
service_id = str(uuid.uuid4())
 
advertise_service(server_socket, "smartbobotServer",
                  service_id = service_id,
                  service_classes = [service_id, SERIAL_PORT_CLASS],
                  profiles = [SERIAL_PORT_PROFILE])

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
CTRL-C to quit
---------------------------

"""

print(msg)




try:
    ## set serial communication
    port  = "/dev/smart_robot"     #port = "" for linux
    baud  = 115200
    robot = smart_robot(port,baud)
    robot.connect()  

    print('Push down [ctrl + c] to stop  ')
    while True:
        print('Wait channel {} of RFCOMM for connecting   '.format(port))
        client_socket, client_info = server_socket.accept()
        print('accept {} for connecting'.format(client_info))
        distance = robot.ultrasonic()
        try:
            while True:
                data = client_socket.recv(1024).decode().lower()
                if len(data) == 0:
                    break
                if data == 'w':
                    robot.go()
                elif data == 's':
                    robot.stop()
                elif data == 'x':
                    robot.back()
                elif data == 'a':
                    robot.left()
                elif data == 'd':
                    robot.right()
                elif data == 'q':
                    robot.rotate_counterclockwise()
                elif data == 'e':
                    robot.rotate_clockwise()
                else:
                    print('Unknown request: {}'.format(data))
        except IOError:
            pass
        client_socket.close()
        print('disconnect')
except KeyboardInterrupt:
    print('disconnect')
finally:
    if 'client_socket' in vars():
        client_socket.close()
    server_socket.close()
    print('disconnect')
