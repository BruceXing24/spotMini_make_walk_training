import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from .tools import PID
from .tools import pose_calculate as pose
from .tools import spot_leg
from .tools import  CPGenerator
import time
from motor_msg.msg import MotorAngles2T


class pose_controller(Node):
    def __init__(self,name):
        super(pose_controller, self).__init__(name)
        self.get_logger().info("spot_controller starts")
        self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.angels_publisher = self.create_publisher(MotorAngles2T,'/angles_on_motor',1)
        self.rpySubscriber = self.create_subscription(Vector3,'rpy_angle',self.subRpy_callback,1)

        self.roll_fromJoy = 0.0
        self.pitch_fromJoy = 0.0
        self.yaw_fromJoy = 0.0
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.PD_r = PID.PID(0.1, 0, 0.2)
        self.PD_p = PID.PID(0.01, 0 ,0.1)
        self.PD_reset = PID.PID(0.02, 0, 0.01)

        self.control_fre = 50.
        self.create_timer(0.02, self.timer_callback)
        self.leg_controller = spot_leg.Leg()
        self.Angles = MotorAngles2T()
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

        self.gait_generator = CPGenerator.CPG(step_length=0.025,ground_clearance=0.025,ground_penetration=0.005,Tstance=0.3,Tswing = 0.2)

        self.stand_pose = np.array([0,45,-90, 0,45,-90, 0,45,-90, 0,45,-90])
        self.lie_down = np.array([60,45,-120, -60,45,-120, 60,45,-120, -60,45,-120])
        self.roll2pitch  =  [0,0]


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
            self.get_logger().info(f'quit to Menu......')

        if self.pose_flag ==True:
            self.roll_fromJoy  =  -joy.axes[0]
            self.pitch_fromJoy =  -joy.axes[1]
            self.yaw_fromJoy   =   joy.axes[6]
            self.roll_angle += self.roll_fromJoy/2
            self.pitch_angle += self.pitch_fromJoy/2
            self.yaw_angle += self.yaw_fromJoy/2

            # print(f'self.roll_angle = {self.roll_angle}')
            # print(f'self.pitch_angle = {self.pitch_angle}')

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

    def subRpy_callback(self,msg:Vector3):
        self.roll2pitch[0] = msg.x
        self.roll2pitch[1] = msg.y



    def timer_callback(self):
        # print(f"walk_flag=={self.walk_flag}")
        # print(f"quit_to_stand=={self.quit_to_stand}")
        # print(f"pose_flag=={self.pose_flag}")
        # print(f"stand_flag=={self.stand_flag}")
        # print(f"lieDown_flag=={self.lieDown_flag}")
        if self.walk_flag==True:
            angles = self.gait_generator.trot_generator(self.gait_generator.trot_timer,'straight',0.8)
            # print(angles)
            angles = np.float32(angles)
            self.gait_generator.trot_timer += 0.02
            self.Angles.angles = angles

            self.Angles.time  = int(10)
            self.angels_publisher.publish(self.Angles)


        elif self.pose_flag == True:
            if self.selfBlacingFlag == False:
                if not self.reset_flag:
                    self.roll_angle   +=   self.roll_fromJoy/2
                    self.pitch_angle  +=   self.pitch_fromJoy/2
                    self.yaw_angle    +=   self.yaw_fromJoy/2
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
                # print(f"ori==={np.array(ori)*180/np.pi}")
                if (np.abs(self.roll2pitch[1]) ) > 3  :
                    self.get_logger().info(f'PD controller start to balance tobot')
                    # self.roll_angle = self.PD_r.PD_controller( self.roll2pitch[0], self.roll_angle)*1.5
                    print(f'self.pitch angle_1 = {self.pitch_angle}')

                    self.pitch_angle = self.PD_p.PD_controller(-self.roll2pitch[1], self.pitch_angle)

                    print(f'self.pitch angle_2 = {self.pitch_angle}')
                    print(f'self.pitch receive angle = {self.roll2pitch[1]}')

                    self.roll_angle = np.clip(self.roll_angle, -30, 30)
                    self.pitch_angle = np.clip(self.pitch_angle, -30, 30)
                    self.yaw_angle = np.clip(self.yaw_angle, -30, 30)
            # get matrix
            matrix_ABs = pose.get_AB(self.roll_angle, self.pitch_angle, self.yaw_angle).T
            # from matrix to angles
            angles = pose.get_angles(matrix_ABs,self.leg_controller)

            self.Angles.angles = np.float32(angles)
            if self.selfBlacingFlag == False:
                self.Angles.time = int(15)
            else:
                self.Angles.time = int(50)
                time.sleep(0.05)


            self.angels_publisher.publish(self.Angles)




        elif self.stand_flag == True :
            self.Angles.angles = np.float32(self.stand_pose)
            self.Angles.time = int(1000)
            self.angels_publisher.publish(self.Angles)
            self.stand_flag = False

        elif self.lieDown_flag == True:
            print("send lie down")
            self.Angles.angles = np.float32(self.lie_down)
            self.Angles.time = int(1000)
            self.angels_publisher.publish(self.Angles)
            self.lieDown_flag = False

        if self.quit_to_stand== True:
            self.walk_flag=False
            self.pose_flag=False
            self.quit_to_stand=False
            self.lieDown_flag = False
            self.stand_flag = False
            self.reset_parameter()



        self.rpy_vector.x = self.roll_angle
        self.rpy_vector.y = self.pitch_angle
        self.rpy_vector.z = self.yaw_angle
        # self.rpyPubliser.publish(self.rpy_vector)

    def reset_parameter(self):
        self.gait_generator.reset_time()
        self.roll_angle = 0.
        self.pitch_angle = 0.
        self.yaw_angle = 0.
        self.roll2pitch= [0,0]


def main():
    rclpy.init()
    node = pose_controller("spot_controller")
    while(1):
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
