#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import TargetPose
from interfaces.msg import PoseReached
from geometry_msgs.msg import Twist
import math
import numpy as np
import time
#class,ki podeduje iz node class od rclpy
class myNode(Node):

    #constructor
    def __init__(self):
        #node name
        super().__init__("interpolator")
        #display log in the terminal
        self.get_logger().info("interpolator started")
        #---------------------------------------------
         #initial values
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.max_acc_time=0
        self.max_velocity=0.5
        self.max_acc=0.05
        self.acc_distance=0
        self.remain_d=0
        self.lin_velocities=[]
        self.ang_velocities=[]
        #nastavi glede na izbran timer
        self.frequency=1 #preveri!!!!!
        self.distance=0.0
        self.rotation=0.0
        self.linDone=0
        self.angDone=0
        #check message
        self.received=False
        #target reached
        self.target_reached=False
        # Initialize robot's current pose and orientation
        self.current_pose_x=0.0
        self.current_pose_y=0.0
        self.current_pose_theta=0.0
        #---------------------------------------------
        #subscriber
        self.pose_subscriber =self.create_subscription(TargetPose, "/target_pose", self.pose_callback, 10)
         #publisher, to cmd_vel topic
        self.send_cmd_=self.create_publisher(Twist, "/cmd_vel", 10)
        #publisher to pose reached
        self.send_reached=self.create_publisher(PoseReached, "/reached_pose", 10)
        #send position after 1 s
        self.timer = self.create_timer(1, self.robot_move)


    #callback, when message received
    def pose_callback(self, msg: TargetPose):
        #self.get_logger().info(str(msg))
        self.x = msg.targetpose.position.x
        self.y = msg.targetpose.position.y
        self.theta = msg.targetorientation
        self.get_logger().info("Pose received")
        self.received=True

    # Function to generate a trapezoidal velocity profile
    def generate_velocity_profile(self, distance):
        
        # Calculate the time required to reach the target velocity during acceleration and deceleration phases (we assumed equal acceleration and deceleration)
        self.max_acc_time = self.max_velocity / self.max_acc
        #acceleration phase
        self.acc_distance=0.5*self.max_acc*self.max_acc_time**2
        #remaining distance at constant speed
        self.remain_d=distance-2*self.acc_distance

        const_vel_time = 0.0
        total_time = 0.0
        acc_time = 0.0

         # Calculate the distance covered during the acceleration and deceleration phases
        if self.remain_d > 0:

            # Calculate the time required for the constant velocity phase
            const_vel_time = self.remain_d / self.max_velocity
            acc_time = self.max_acc_time
        else:
            const_vel_time = 0.0
            acc_time = np.sqrt(2*distance / self.max_acc)

        #calculate total time needed
        total_time=2*acc_time+const_vel_time
        self.get_logger().info("total time is {}".format(total_time))
        #N of points needed
        N_points=int(total_time*self.frequency)
        #time increment
        dt=1.0/self.frequency

        # Generate the trapezoidal velocity profile
        velocities=[]
        velocity = 0.0
        for i in range(N_points):
            t = i * dt  # Time
            # Calculate the velocity at time t based on the trapezoidal profile
            

            if t <= acc_time:
                velocity = self.max_acc * t  # Acceleration phase
            elif t > acc_time and (t <= acc_time + const_vel_time):
                velocity = self.max_velocity  # Constant velocity phase
            else:
                deceleration_time_offset = t - (total_time - self.max_acc_time)
                velocity = self.max_velocity - self.max_acc * deceleration_time_offset  # Deceleration phase
            velocities.append(velocity)
            #preveri, če je izpis ok
            self.get_logger().info("calculated velocities {}".format(velocities))
        return velocities
        
    #robot move callback
    def robot_move(self):

        if self.received:
            distance=math.hypot(self.x - self.current_pose_x, self.y - self.current_pose_y)
            rotation=math.atan2(self.y-self.current_pose_y, self.x-self.current_pose_x)
            #move straight
            if self.linDone==0 and self.angDone==0:
                self.move_straight(distance)
                self.linDone=1
            
            if self.angDone ==0.0 and self.linDone ==1:
                self.turn(rotation)
                self.angDone=1
            #stop the move
            if self.linDone==1 and self.angDone==1:
                #publich to reached topic
                msg=PoseReached()
                msg.posereached=True
                self.send_reached.publish(msg)
                self.received=False
                self.get_logger().info("target reached")
                self.linDone=0
                self.angDone=0
    
    #straight move
    def move_straight(self, distance):
        self.lin_velocities=self.generate_velocity_profile(distance)
        self.get_logger().info("driving straight")

        elapsed_total_time = 0.0

        for vel in self.lin_velocities:
            start = time.time()
            speed=Twist()
            speed.linear.x = vel
            self.send_cmd_.publish(speed)
            end = time.time()

            elapsed_time = end - start
             # Calculate the remaining time to sleep to achieve the desired frequency
            sleep_duration = 1 / self.frequency
            time.sleep(sleep_duration)
            elapsed_total_time += 1 / self.frequency
            message = "Elapsed time: " + str(elapsed_total_time)
            self.get_logger().info(message)

        # Stop the robot after reaching the target pose
        speed.linear.x = 0.0
        self.send_cmd_.publish(speed)

    #rotation
    def turn(self, rotation):
        self.ang_velocities=self.generate_velocity_profile(rotation)
        self.get_logger().info("turning")

        elapsed_total_time=0.0
        for vel in self.ang_velocities:
            start = time.time()
            speed=Twist()
            speed.angular.z = vel
            self.send_cmd_.publish(speed)
            end = time.time()

            elapsed_time = end - start
             # Calculate the remaining time to sleep to achieve the desired frequency
            sleep_duration = 1 / self.frequency
            time.sleep(sleep_duration)
            elapsed_total_time += 1 / self.frequency
            message = "Elapsed time: " + str(elapsed_total_time)
            self.get_logger().info(message)

        #stop the robot after the target angle
        speed.angular.z = 0.0
        self.send_cmd_.publish(speed)

#---------------------------------
def main(args=None):
    #initialize ros2 communication
    rclpy.init(args=args)
    #--------------------------------------------------------
    #CODE
    #create node
    node = myNode()
    #to run node indefinitely (stop it with ctrl+c)
    rclpy.spin(node)
    #sleep to maintain loop rate
    #--------------------------------------------------------
    #destroy node and shut down communication
    rclpy.shutdown()

if __name__=='__main__':
    main()