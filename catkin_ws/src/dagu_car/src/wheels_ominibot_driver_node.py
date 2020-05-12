#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.smart_robotV12_driver import smart_robotV12 #DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        self.port = "/dev/smart_robot_omnibotV12"
        self.baud = 115200

        # Setup publishers
        self.driver = smart_robotV12(self.port,self.baud)
        self.driver.connect()
        self.driver.set_mode(3) ## Start smartbot and choose " 0 :omnibot" , 1:Mecanum ,2: normal motor with encoder , 3: normal motor without encoder"
        
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.TT_motor_knightcar(left=0.0,right=0.0)
            return
        self.driver.TT_motor_knightcar(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()  
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.driver.TT_motor_knightcar(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
