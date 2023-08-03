import rclpy
from rclpy.node import Node
import torch
from stable_baselines3 import PPO
from motor_msg.msg import MotorAngles2T
import numpy as np
from .CPGenerator import CPG
from .spot_leg import Leg
from geometry_msgs.msg import Vector3
import time
from sensor_msgs.msg import Joy

class PPO_controller(Node):
    def __init__(self,name):
        super(PPO_controller,self).__init__(name)
        self.contorl_frequcy = 200.
        self.create_timer(1./ self.contorl_frequcy,callback=self.control_callback)
        self.angels_publisher = self.create_publisher(MotorAngles2T,'/angles_on_motor',1)
        self.Angles = MotorAngles2T()
        self.AI = PPO.load('src/robot_state/model/train_result_7m_slope_PPO2_6000000_steps.zip')
        self.initial_angles = np.array([0,45,-90, 0,45,-90, 0,45,-90, 0,45,-90])
        
        # for observation
        self.gait_generator = CPG(step_length=0.025, ground_clearance=0.025, ground_penetration=0.01)
        self.gait_timer = 0.
        self.referFactor = 1.
        self.optiFactor  = 1.

        self.opti_range_upper = 4. 
        self.opti_range_knee = -6.
        self.angles_on_motor = np.array([0,45,-90, 0,45,-90, 0,45,-90, 0,45,-90])

        self.create_timer(0.01, self.control_callback)
        self.obs_space =  np.array([0]*11)      

        self.create_subscription(Vector3,'/rpy_angle',self.rpy_callback,5)
        self.create_subscription(Vector3,'/Wxyz',self.Wxyz_callback,5)
        self.rpy = Vector3()
        self.Wxyz = Vector3()
        self.calia_yaw_list = []
        self.calia_yaw= 0
        self.calia_yaw_flag = False


        self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.AI_flag = False
        self.get_logger().info(f'you are under stand mode')
        self.stand_flag =True




    def control_callback(self):
        # reset() get initial obs
        self.obs_space[6:8]  =  self.initial_angles [1:3] * np.pi/180    
        self.obs_space[8:10] =  self.initial_angles [4:6] * np.pi/180           # obs in radius
        self.obs_space[10]  =  0.                


        if len(self.calia_yaw_list)<200:
            self.get_logger().info('yaw is calibrating,do not move robot')
            self.calia_yaw_list.append(self.rpy.z)

        elif len(self.calia_yaw_list)==200 and self.calia_yaw_flag==False :
            self.calia_yaw =np.sum( np.array(self.calia_yaw_list))/200
            self.calia_yaw_flag = True
        

        #

        if self.stand_flag == True :
            self.Angles.angles = np.float32(self.initial_angles)
            self.Angles.time = int(1000)
            self.angels_publisher.publish(self.Angles)
            self.stand_flag = False

        if self.AI_flag ==True:
            # if self.calia_yaw_flag ==True:
            # while true
            print("under AI model")
            action = self.AI.predict(self.obs_space)
            # step-----------step return obs, reward,done
            refer_angles = self.gait_generator.trot_generator(self.gait_timer,direction='straight',turn_radius=1.)
            self.angles_on_motor = self.merge_action(action,refer_angles)         # in degree   
            
            angles= np.array(self.angles_on_motor,dtype=np.float32)
            self.Angles.time  = int(4)
            self.Angles.angles = angles
            self.angels_publisher.publish(self.Angles)
            #  publish  to motor # 
            self.gait_timer += 1./self.contorl_frequcy
            

            # read imu data
    
            self.obs_space[0:3] = [self.rpy.x,  self.rpy.y,  self.rpy.z-self.calia_yaw]
            self.obs_space[3:6] = [self.Wxyz.x, self.Wxyz.y, self.Wxyz.z]
            self.obs_space[6:8] = angles[1:3]*np.pi/180    
            self.obs_space[8:10] = angles[4:6]*np.pi/180           # obs in radius
            self.obs_space[10]  =  self.gait_timer            
                # print(f'yaw ==={self.rpy.z-self.calia_yaw}')

        # in degree
        #  need to know the size of action



    def merge_action(self,action,refer_angles):
        #print(action)
        # in degree
        LF = [0, 0, 0]
        RF = [0, 0, 0]
        LB = [0, 0, 0]
        RB = [0, 0, 0]

        LF [1] = RB [1] = action[0][0] * self.opti_range_upper * np.sin(self.gait_generator.trot_timer)
        LF [2] = RB [2] = action[0][1] * self.opti_range_knee * np.sin(self.gait_generator.trot_timer)
        RF [1] = LB [1] = action[0][2] * self.opti_range_upper * np.sin(self.gait_generator.trot_timer)
        RF [2] = LB [2] = action[0][3] * self.opti_range_knee * np.sin(self.gait_generator.trot_timer)
        return np.hstack((LF, RF, LB, RB)) * self.optiFactor + refer_angles * self.referFactor



    def rpy_callback(self,rpy:Vector3):
        self.rpy.x = rpy.x
        self.rpy.y = rpy.y
        self.rpy.z = rpy.z

    def Wxyz_callback(self, Wxyz:Vector3):
        self.Wxyz.x = Wxyz.x
        self.Wxyz.y = Wxyz.y
        self.Wxyz.z = Wxyz.z


    def joy_callback(self,joy:Joy):
        # print(f'changemode=={self.change_mode_flag}')
        if joy.buttons[0]==1:
            self.AI_flag = False
            self.get_logger().info(f'you are under stand mode')
            self.stand =True
        
        elif joy.buttons[1] ==1:
            self.AI_flag = True
            self.get_logger().info(f'you are under AI mode')
            self.stand =False


def main():
    rclpy.init()
    ppo_controller = PPO_controller("ppo_controller")
    rclpy.spin(ppo_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




