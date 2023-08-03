# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msg.msg import MotorAngles2T
import pybullet as p
import pybullet_data as pd
from .tools import spot_leg
import numpy as np

class ShowRobot(Node):
      def __init__(self,name):
            super().__init__(name)
            self.get_logger().info("pybullet show is created")
            # self.create_timer(0.05,self.timer_callback)
            self.create_timer(1.0/240.,self.timer_callback_2)
            self.create_subscription(MotorAngles2T,'/angles_on_motor',self.sub_callback,1)
            # fig = plt.figure()
            self.count = 0
            self.leg_controller = spot_leg.Leg()

            self.robot = p.loadURDF("/src/Pose/urdf/spot_old.urdf", [0, 0, 0.4],
                                    useMaximalCoordinates=False,
                                    flags=p.URDF_USE_IMPLICIT_CYLINDER,
                                    baseOrientation = p.getQuaternionFromEuler([0, 0, np.pi]))
            self.planeID = p.loadURDF("plane.urdf")

            self.stand_pose = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]

            self.angles_on_motor = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]


      def sub_callback(self,msg):
            sub_angles = msg.angles
            self.angles_on_motor[0] = np.array(sub_angles[0:3])*np.pi/180
            self.angles_on_motor[1] = np.array(sub_angles[3:6])*np.pi/180
            self.angles_on_motor[2] = np.array(sub_angles[6:9])*np.pi/180
            self.angles_on_motor[3] = np.array(sub_angles[9:12])*np.pi/180



      def timer_callback_2(self):
            if self.count < 100:
                  p.setGravity(0, 0, 0)
                  self.leg_controller.positions_control2(self.robot, self.stand_pose[0],self.stand_pose[1],self.stand_pose[2],self.stand_pose[3])

            if self.count > 200 and self.count<500:
                  p.setGravity(0, 0, -9.8)
                  self.leg_controller.positions_control2(self.robot, self.stand_pose[0],self.stand_pose[1],self.stand_pose[2],self.stand_pose[3])

            if self.count >500:
                  self.leg_controller.positions_control2(self.robot,self.angles_on_motor[0],self.angles_on_motor[1],self.angles_on_motor[2],self.angles_on_motor[3])
            p.stepSimulation()
            self.count += 1

def main():
      p.connect(p.GUI)
      p.setAdditionalSearchPath(pd.getDataPath())



      rclpy.init()
      node = ShowRobot("show_robot")
      rclpy.spin(node)
      rclpy.shutdown()      




if __name__=='__main__':
      main()