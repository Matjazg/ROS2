#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#import message
from interfaces.msg import TargetPose
from interfaces.msg import PoseReached
from geometry_msgs.msg import Pose
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import math
import time

class command_node(Node):

    #constructor
    def __init__(self):
        #node name
        super().__init__("command_node")
        #initial values
        self.reached=True
        self.x_array=np.array((1.0,1.0,1.0,1.0))
        self.y_array=np.array((0.0,0.0,0.0,0.0))
        self.theta_array=np.array((90.0,90.0,90.0,90.0))
        self.current_index=0
        #subscriber to pose reached
        self.pose_reached =self.create_subscription(PoseReached, "/reached_pose", self.reached_callback, 10)
        #publisher, define message and your topic
        self.command_pub_=self.create_publisher(TargetPose, "/target_pose", 10)
        #display log in the terminal
        self.get_logger().info("Command node started")
        #publish every 0.1s
        self.timer=self.create_timer(0.1, self.timer_callback)
    
    #reached pose callback
    def reached_callback(self, msg: PoseReached):
        print(msg)
        if msg.posereached:
            self.reached=True
            self.get_logger().info("pose {} reached".format(self.current_index))

    #define message parameters
    def timer_callback(self):
         #check if pose is reached and array index is not exceeded
        if self.current_index<(len(self.x_array)) and self.reached==True:
            #send pose and orientation from array
            x=self.x_array[self.current_index]
            y=self.y_array[self.current_index]
            theta=self.x_array[self.current_index]
            #publish message
            msg1=TargetPose()
            msg1.targetpose.position.x=x
            msg1.targetpose.position.y=y
            msg1.targetorientation=math.radians(theta)
            self.command_pub_.publish(msg1)
            self.get_logger().info("Publishing: {}".format(msg1))
            self.reached=False
            #increase index by 1
            self.current_index+=1
        #stop publishing
        elif self.current_index ==len(self.x_array):
            self.timer.cancel()
            self.get_logger().info("stopped publishing")
        
def main(args=None):
    #initialize ros2 communication
    rclpy.init(args=args)
    node=command_node()
    #rclpy.spin(node)
    rclpy.spin(node)

    #--------------------------------------------------------
    #CODE
    
    #--------------------------------------------------------
    #destroy node and shut down communication
    #node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
