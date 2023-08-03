import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Vector3
from . import PID
from . import pose_calculate as pose
from . import spot_leg
import time
from motor_msg.msg import MotorAngles


class pose_controller(Node):
    def __init__(self,name):
        super(pose_controller, self).__init__(name)
        self.get_logger().info("spot_controller starts")
        self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.angels_publisher = self.create_publisher(MotorAngles,'/angles_on_motor',1)
        self.rpyPubliser = self.create_publisher(Vector3,'rpy_angle',1)

        self.roll_fromJoy = 0.0
        self.pitch_fromJoy = 0.0
        self.yaw_fromJoy = 0.0
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.PD_r = PID.PID(0.1, 0, 0.2)
        self.PD_p = PID.PID(0.05, 0 ,0.1)
        self.PD_reset = PID.PID(0.02, 0, 0.01)


        self.create_timer(0.02,self.timer_callback)
        self.leg_controller = spot_leg.Leg()
        self.Angles = MotorAngles()
        self.pre_Angles = np.array([0,0,0, 0,0,0, 0,0,0, 0,0,0])

        self.rpy_vector = Vector3()
        self.selfBlacingFlag = False

        self.reset_flag = False
        self.lieDown_flag = False

        self.stand_flag = True
        self.walk_flag = False
        self.pose_flag = False
        self.quit_to_stand = False
        self.change_mode_flag = True





        self.stand_pose = np.array([0,45,-90, 0,45,-90, 0,45,-90, 0,45,-90])
        self.lie_down = np.array([60,45,-120, -60,45,-120, 60,45,-120, -60,45,-120])
    def joy_callback(self,joy:Joy):
        # print(f'changemode=={self.change_mode_flag}')
        if self.change_mode_flag ==True:
            if joy.buttons[0]==1:
                self.walk_flag = True
                self.get_logger().info(f'you are under walk mode')
                self.change_mode_flag =False
            elif joy.buttons[1] ==1:
                self.pose_flag = True
                self.get_logger().info(f'you are under pose mode')
                self.change_mode_flag =False
            elif joy.buttons[2] ==1:
                self.stand_flag = True
                self.get_logger().info(f'you are under stand mode')
                self.change_mode_flag =False
            elif joy.buttons[3]==1:
                self.lieDown_flag = True
                self.get_logger().info(f'you are under lie down_mode')
                self.change_mode_flag =False
        elif joy.buttons[6]==1:
            self.quit_to_stand = True
            self.change_mode_flag = True
            self.get_logger().info(f'quit to Menu')

        if self.pose_flag ==True:
            self.roll_fromJoy  =  -joy.axes[0]
            self.pitch_fromJoy =  -joy.axes[1]
            self.yaw_fromJoy   =   joy.axes[6]
            self.roll_angle += self.roll_fromJoy/3
            self.pitch_angle += self.pitch_fromJoy/3
            self.yaw_angle += self.yaw_fromJoy/3
            if joy.buttons[2]==1:
                self.reset_flag = True
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
        if self.walk_flag==True:
            pass
        elif self.pose_flag == True:
            if self.selfBlacingFlag == False:
                if not self.reset_flag:
                    self.roll_angle += self.roll_fromJoy/2
                    self.pitch_angle += self.pitch_fromJoy/2
                    self.yaw_angle += self.yaw_fromJoy/2
                else:
                    self.get_logger().info("reseting")
                    self.roll_angle  = self.PD_reset.PD_controller( 0,  self.roll_angle  )
                    self.pitch_angle = self.PD_reset.PD_controller( 0,  self.pitch_angle )
                    self.yaw_angle   = self.PD_reset.PD_controller( 0,  self.yaw_angle   )
                    if np.abs(self.roll_angle) < 2 and np.abs(self.pitch_angle) < 2 and np.abs(self.yaw_angle) < 2:
                        self.get_logger().info("reset done")
                        self.get_logger().info("now you are in pose mode")
                        self.reset_flag = False
                self.roll_angle = np.clip(self.roll_angle, -25, 25)
                self.pitch_angle = np.clip(self.pitch_angle, -25, 25)
                self.yaw_angle = np.clip(self.yaw_angle, -25, 25)
            # print(f'roll={self.roll_fromJoy},pitch={self.pitch_fromJoy},yaw={self.yaw_fromJoy}')
            # print(f'roll={self.roll_angle},pitch={self.pitch_angle},yaw={self.yaw_angle}')
            elif self.selfBlacingFlag==True:
                print("under self balance")
                ori=[0,0]
                # print(f"ori==={np.array(ori)*180/np.pi}")
                if np.abs(ori[0]*180/np.pi+ori[1]*180/np.pi) *180/np.pi > 2  :
                    self.get_logger().info(f'PD controller start to balance tobot')
                    self.roll_angle = self.PD_r.PD_controller( ori[0]*180/np.pi, self.roll_angle)*1.5
                    self.pitch_angle = self.PD_p.PD_controller(-ori[1]*180/np.pi, self.pitch_angle)

                    self.roll_angle = np.clip(self.roll_angle, -30, 30)
                    self.pitch_angle = np.clip(self.pitch_angle, -30, 30)
                    self.yaw_angle = np.clip(self.yaw_angle, -30, 30)
            matrix_ABs = pose.get_AB(self.roll_angle, self.pitch_angle, self.yaw_angle).T
            x1, y1, z1 = matrix_ABs[0, 0], matrix_ABs[0, 1], matrix_ABs[0, 2]
            x2, y2, z2 = matrix_ABs[1, 0], matrix_ABs[1, 1], matrix_ABs[1, 2]
            x3, y3, z3 = matrix_ABs[2, 0], matrix_ABs[2, 1], matrix_ABs[2, 2]
            x4, y4, z4 = matrix_ABs[3, 0], matrix_ABs[3, 1], matrix_ABs[3, 2]
            theta1, theta2, theta3    = self.leg_controller.IK_L_2(x1+0.0113, y1, z1)
            theta4, theta5, theta6    = self.leg_controller.IK_R_2(x2+0.0113, y2, z2)
            theta7, theta8, theta9    = self.leg_controller.IK_L_2(x3+0.0113, y3, z3)
            theta10, theta11, theta12 = self.leg_controller.IK_R_2(x4+0.0113, y4, z4)
            angles  = np.array([theta1, theta2, theta3,theta4, theta5, theta6,theta7, theta8, theta9,theta10, theta11, theta12])*180/np.pi
            self.Angles.angles = np.float32(angles)
            self.angels_publisher.publish(self.Angles)




        elif self.stand_flag == True :
            self.Angles.angles = np.float32(self.stand_pose)
            self.angels_publisher.publish(self.Angles)
            self.stand_flag = False

        elif self.lieDown_flag == True:
            print("send lie down")
            self.Angles.angles = np.float32(self.lie_down)
            self.angels_publisher.publish(self.Angles)
            self.lieDown_flag = False

        if self.quit_to_stand== True:
            self.walk_flag=False
            self.pose_flag=False
            self.quit_to_stand=False
            self.lieDown_flag = False
            self.stand_flag = False


        # print(f"walk_flag=={self.walk_flag}")
        # print(f"quit_to_stand=={self.quit_to_stand}")
        # print(f"pose_flag=={self.pose_flag}")
        # print(f"stand_flag=={self.stand_flag}")
        # print(f"lieDown_flag=={self.lieDown_flag}")

        self.rpy_vector.x = self.roll_angle
        self.rpy_vector.y = self.pitch_angle
        self.rpy_vector.z = self.yaw_angle
        self.rpyPubliser.publish(self.rpy_vector)



def main():
    rclpy.init()
    node = pose_controller("spot_controller")
    while(1):
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
