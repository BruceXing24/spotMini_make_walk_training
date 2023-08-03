# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from . import spot_in_3D
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
from motor_msg.msg import MotorAngles
import pybullet as p
import pybullet_data as pd
from . import spot_leg
import numpy as np

class ShowRobot(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("graph is created")
        self.create_timer(0.05,self.timer_callback)

        self.create_subscription(Vector3,"/rpy_angle",self.rpy_callback, 1)
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        # fig = plt.figure()
        self.ax = plt.axes(projection='3d')
        self.count = 0
        self.leg_controller = spot_leg.Leg()


        self.stand_pose = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]

        self.angles_on_motor = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]

    def rpy_callback(self,vector:Vector3):
        self.roll_angle = vector.x
        self.pitch_angle = vector.y
        self.yaw_angle = vector.z


    def timer_callback(self):
        spot_in_3D.show(self.roll_angle,self.pitch_angle,self.yaw_angle,self.ax)



def main():

    rclpy.init()
    node = ShowRobot("show_robot")
    rclpy.spin(node)
    rclpy.shutdown()




if __name__=='__main__':
    main()