#!/usr/bin/python

from Adafruit_MotorHAT.Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT as DC_motor
import time  ,sys ,select,termios ,tty ,atexit
import numpy as np

## ==== set parameter ====

board = {}
board["addr"] = 0x60

servo_parameter_text = ["frequency" ,"pulse_max" ,"pulse_min" ,"angle_max" , "angle_min","exercise_time" , "bottom_angle","arm_angle"  ,"head_angle" , "paw_angle", "step_angle" ,"move_status"]

servo_parameter = {
                   servo_parameter_text[0]    : 50   ,  # frequency
                   servo_parameter_text[1]    : 2.0  ,  # pulse max
                   servo_parameter_text[2]    : 1.0  ,  # pulse min
                   servo_parameter_text[3]    : 180.0,  # angle max
                   servo_parameter_text[4]    : 0.0  ,  # angle min
                   servo_parameter_text[5]    : 0.5  ,  # exercise time
                   servo_parameter_text[6]    : 90.0 ,  # bottom angle
                   servo_parameter_text[7]    : 35.0  , # arm angle
                   servo_parameter_text[8]    : 0.0  ,  # head angle
                   servo_parameter_text[9]    : 0.0  ,  # paw angle
                   servo_parameter_text[10]   : 1.0  ,  # step angle
                   servo_parameter_text[11]   : False,  # move status
                  }

## ==== set pin of servo ====

pin = {}
pin[ servo_parameter_text[6] ] = 15 
pin[ servo_parameter_text[7] ] = 0
pin[ servo_parameter_text[8] ] = 14
pin[ servo_parameter_text[9] ] = 1

## ==== set pin of servo ====

servo = PWM(board["addr"])
servo.setPWMFreq(servo_parameter["frequency"])

def setServoPulse( pin, angle):

    pulseLength = 1000000                         # 1,000,000 us per second
    pulseLength /= servo_parameter["frequency"]   # Hz
    pulseLength /= 4096                            # 12 bits of resolution
    pulse = (servo_parameter["pulse_max"]  - servo_parameter["pulse_min"] ) * (angle - servo_parameter["angle_min"] ) / ( servo_parameter["angle_max"] - servo_parameter["angle_min"] ) + servo_parameter["pulse_min"]
    pulse *= 1000
    pulse /= pulseLength
    servo.setPWM( pin , 0 , int(pulse) )

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin , termios.TCSADRAIN, settings)
    return key

def constrain(input , low , high):

    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input

def check_value(value):
    value = constrain( value ,  servo_parameter["angle_min"]   , servo_parameter["angle_max"]  )
    return value

def servo_release(pin):
    servo.setPWM( pin , 0 , 0 )

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    key_word = {
               "w": servo_parameter_text[7]  ,    # arm angle
               "x": servo_parameter_text[7]  ,    # arm angle
               "a": servo_parameter_text[6]  ,    # bottom angle
               "d": servo_parameter_text[6]  ,    # bottom angle
               "s": servo_parameter_text[11] ,    # move status
               "q": servo_parameter_text[8]  ,    # head angle
               "e": servo_parameter_text[8]  ,    # head angle
               "z": servo_parameter_text[9]  ,    # paw angle
               "c": servo_parameter_text[9]       # paw angle
              }
    
    for start_text in (servo_parameter_text[6] , servo_parameter_text[7],servo_parameter_text[8],servo_parameter_text[9]):
        setServoPulse(pin[start_text] , servo_parameter[start_text])    


    while True:
        key = getKey()
        if key in key_word.keys() :
            if key == "s":
                for control_pin in pin.values():
                    servo_release(control_pin)
                    servo_parameter["move_status"] = False
                print ("Release all servo ")
            else:
                global control_text
                control_text = key_word[ key ]
                if key in ("x","d","e","z"):
                    servo_parameter[control_text] = check_value(servo_parameter[control_text] - servo_parameter["step_angle"])
                else:
                    servo_parameter[control_text] = check_value(servo_parameter[control_text] + servo_parameter["step_angle"])
                print(" {} now is {}".format( control_text , servo_parameter[control_text]) )
                servo_parameter["move_status"] = True
        else:
            if key == '\x03':
                break
        

        if servo_parameter["move_status"] == True:
            setServoPulse(pin[control_text] , servo_parameter[control_text])
            servo_parameter["move_status"] == False
            



    termios.tcsetattr(sys.stdin , termios.TCSADRAIN, settings)















