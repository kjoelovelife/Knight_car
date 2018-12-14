#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    select_color = data.data
    trigger = { 'R': (2,1),'G': (3,1),'B': (4,1),'r': (2,1),'g': (3,1),'b': (4,1) }
    close   = {'C':(9,10,11,0),'c':(9,10,11,0)}
    if select_color in trigger.keys() :
        print 'The color is {} and Pin is {} '.format(select_color, trigger[select_color][0]) 
        pin   = trigger[select_color][0]
        digital_signal =trigger[select_color][1]   
    elif select_color in close.keys():   
        pin   = close[select_color][0:3]
        digital_signal = close[select_color][3]   
        print 'Close all.'   
    

def listener():
    
    rospy.init_node('subcriber_teleop', anonymous=True)
    rospy.Subscriber("pub_teleop", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
