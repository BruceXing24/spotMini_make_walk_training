# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pybullet as p
import numpy as np
from sensor_msgs.msg import Joy
import pybullet_data as pd
from geometry_msgs.msg import Vector3
from . import PID
from . import pose_calculate as pose
from . import spot_leg
import time






class pose_controller(Node):
    def __init__(self,name,table):
        super(pose_controller, self).__init__(name)
        self.get_logger().info("pybullet start")
        self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.roll_fromJoy = 0.0
        self.pitch_fromJoy = 0.0
        self.yaw_fromJoy = 0.0

        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        self.PD_r = PID.PID(0.1, 0, 0.2)
        self.PD_p = PID.PID(0.05, 0 ,0.1)
        self.PD_reset = PID.PID(0.02, 0, 0.01)

        self.stand_pose = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]
        self.sit_pose = [[0,0,0],[0,0,0],[0,np.pi/3,-np.pi*2/3],[0,np.pi/3,-np.pi*2/3]]

        self.robot = p.loadURDF("/home/xing/my_mini_spot/src/Pose/urdf/spot_old.urdf",[0,0,1.4],
                                    useMaximalCoordinates=False,
                                    flags=p.URDF_USE_IMPLICIT_CYLINDER,
                                    baseOrientation = p.getQuaternionFromEuler([0, 0, np.pi]),
                                )
        self.planeID = p.loadURDF("plane.urdf")
        self.table = table

        self.count = 0
        self.leg_controller = spot_leg.Leg()

        self.create_timer(0.02,self.timer_callback)
        self.create_timer(0.01,self.pybullet_fre)
        self.rpyPubliser = self.create_publisher(Vector3,'rpy_angle',10)
        self.rpy_vector = Vector3() 
        self.reset_flag = False
        self.table_rotate_flag  = False
        self.angle_delta = 0.01
        self.table_angle = 0
        self.selfBlacingFlag = False




    def joy_callback(self,joy:Joy):
        self.roll_fromJoy  =  -joy.axes[0]
        self.pitch_fromJoy =  -joy.axes[1]
        self.yaw_fromJoy   =    joy.axes[6]
        if joy.buttons[2]==1:
            self.reset_flag = True

        if joy.buttons[3] ==1:
            if self.table_rotate_flag == False:
                self.table_rotate_flag = True
                self.get_logger().info(f'table start to rotate')
                time.sleep(1)
            else:
                self.table_rotate_flag = False
                self.get_logger().info(f'table start not to rotate')
                time.sleep(1)
        if joy.buttons[0]==1:
            if self.selfBlacingFlag == False:
                self.selfBlacingFlag = True
                self.get_logger().info(f'self balancing mode')
                time.sleep(1)
            else:
                self.selfBlacingFlag = False
                self.get_logger().info(f'self balanceing mode stop!')
                time.sleep(1)




    def timer_callback(self):

        
        if self.selfBlacingFlag == False:
            if not self.reset_flag:
                self.roll_angle += self.roll_fromJoy/3
                self.pitch_angle += self.pitch_fromJoy/3
                self.yaw_angle += self.yaw_fromJoy/3
            else:
                self.get_logger().info("reseting")
                self.roll_angle  = self.PD_reset.PD_controller( 0,  self.roll_angle  )
                self.pitch_angle = self.PD_reset.PD_controller( 0,  self.pitch_angle )
                self.yaw_angle   = self.PD_reset.PD_controller( 0,  self.yaw_angle   )
                if np.abs(self.roll_angle) < 2 and np.abs(self.pitch_angle) < 2 and np.abs(self.yaw_angle) < 2:
                    self.get_logger().info("reset done")
                    self.get_logger().info("now you are in pose mode")
                    self.reset_flag = False
            self.roll_angle = np.clip(self.roll_angle, -20, 20)
            self.pitch_angle = np.clip(self.pitch_angle, -20, 20)
            self.yaw_angle = np.clip(self.yaw_angle, -20, 20)
        # print(f'roll={self.roll_fromJoy},pitch={self.pitch_fromJoy},yaw={self.yaw_fromJoy}')
        # print(f'roll={self.roll_angle},pitch={self.pitch_angle},yaw={self.yaw_angle}')


        elif self.selfBlacingFlag==True:
            print("under self balance")
            ori = self.base_info[1]
            ori = p.getEulerFromQuaternion(ori)
            # print(f"ori==={np.array(ori)*180/np.pi}")

            if np.abs(ori[0]*180/np.pi+ori[1]*180/np.pi) *180/np.pi > 2  :
                self.get_logger().info(f'PD controller start to balance tobot')
                self.roll_angle = self.PD_r.PD_controller( ori[0]*180/np.pi, self.roll_angle)*1.5
                self.pitch_angle = self.PD_p.PD_controller(-ori[1]*180/np.pi, self.pitch_angle)

                self.roll_angle = np.clip(self.roll_angle, -30, 30)
                self.pitch_angle = np.clip(self.pitch_angle, -30, 30)
                self.yaw_angle = np.clip(self.yaw_angle, -30, 30)



        if self.table_rotate_flag==True:
            if self.table_angle>=np.pi/6:
                self.angle_delta = -0.001
            
            elif self.table_angle <= -np.pi/6:
                self.angle_delta = 0.001

            self.table_angle+=self.angle_delta
            
            


        self.rpy_vector.x = self.roll_angle
        self.rpy_vector.y = self.pitch_angle
        self.rpy_vector.z = self.yaw_angle
        self.rpyPubliser.publish(self.rpy_vector)
        # self.table_angle = 0


    def pybullet_fre(self):

            self.base_info = p.getBasePositionAndOrientation(self.robot)

            if self.count < 100:
                p.setGravity(0, 0, 0)
                self.leg_controller.positions_control2(self.robot, self.stand_pose[0],self.stand_pose[1],self.stand_pose[2],self.stand_pose[3])

            if self.count > 200 and self.count<500:
                print("initlizing")
                p.setGravity(0, 0, -9.8)
                self.leg_controller.positions_control2(self.robot, self.stand_pose[0],self.stand_pose[1],self.stand_pose[2],self.stand_pose[3])
                if self.count ==499:
                    self.get_logger().info("initliazing done ,now you are in pose mode")
            if self.count >= 500:
                p.setGravity(0, 0, -9.8)
                posi, ori = p.getBasePositionAndOrientation(self.robot)
                # print(f'posi====={posi}')
                matrix_ABs = pose.get_AB(self.roll_angle, self.pitch_angle, self.yaw_angle).T
                x1, y1, z1 = matrix_ABs[0, 0], matrix_ABs[0, 1], matrix_ABs[0, 2]
                x2, y2, z2 = matrix_ABs[1, 0], matrix_ABs[1, 1], matrix_ABs[1, 2]
                x3, y3, z3 = matrix_ABs[2, 0], matrix_ABs[2, 1], matrix_ABs[2, 2]
                x4, y4, z4 = matrix_ABs[3, 0], matrix_ABs[3, 1], matrix_ABs[3, 2]

                theta1, theta2, theta3    = self.leg_controller.IK_L_2(x1+0.0113, y1, z1)
                theta4, theta5, theta6    = self.leg_controller.IK_R_2(x2+0.0113, y2, z2)
                theta7, theta8, theta9    = self.leg_controller.IK_L_2(x3+0.0113, y3, z3)
                theta10, theta11, theta12 = self.leg_controller.IK_R_2(x4+0.0113, y4, z4)
                # if self.count%200 == 0:
                #     # for checking data
                #     print("x1,y1,z1=={},{},{}".format(x1,y1,z1))
                #     print("x2,y2,z2=={},{},{}".format(x2,y2,z2))
                #     print("x3,y3,z3=={},{},{}".format(x3,y3,z3))
                #     print("x4,y4,z4=={},{},{}".format(x4,y4,z4))

                self.leg_controller.positions_control2(self.robot, [theta1, theta2, theta3], [theta4, theta5, theta6],
                                           [theta7, theta8, theta9],[theta10, theta11, theta12])

                
                self.leg_controller.table_control(self.table,self.table_angle,self.table_angle,self.table_angle)
                # print(f'self.table_angle=={self.table_angle}')

                # self.leg_controller.positions_control2(self.robot, self.stand_pose[0],self.stand_pose[1],
                #                                       self.stand_pose[2],self.stand_pose[3])
            self.count += 1
            p.stepSimulation()


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    table = p.loadURDF("/home/xing/my_mini_spot/src/Pose/urdf1/hexapod_v2.urdf", [0, 0, 1], useMaximalCoordinates=False,
            flags=p.URDF_USE_IMPLICIT_CYLINDER,globalScaling=6.0)
    p.changeDynamics(bodyUniqueId=table, linkIndex=-1, mass = 3.0)


    for i in range(17):
        info = p.changeDynamics(bodyUniqueId=table, linkIndex=i+1,mass = 1.0)
        print(info)


    rclpy.init()
    node = pose_controller("spot_pose_controller",table)
    while(1):
        rclpy.spin(node)
        rclpy.shutdown()



if __name__ == '__main__':
    main()
