# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2023/1/12 22:31

import pybullet as p
import numpy as np



class Leg():
    def __init__(self,  shoulder2hip=0.055, hip2knee=0.109, knee2end=0.125):
        self.l1 = shoulder2hip
        self.l2 = hip2knee
        self.l3 = knee2end
        self.Position_Gain = .5
        self.Velocity_Gain = .5
        self.force = 10
        self.Max_velocity = 5
        #                    LF      RF     LB    RB
        self.joint_angle = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.t1 = 0.
        self.t2 = -0.5

    def time_reset(self):
        self.t1 = 0.
        self.t2 = -0.5


    # need to be calibrated
    def IK_R(self, x, y, z):
        D = (x ** 2 + y ** 2 + z ** 2 - self.l1 ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        theta3 = np.arctan2(-np.sqrt(1 - D ** 2), D)
        theta1 = -np.arctan2(-z, y) - np.arctan2(np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2), -self.l1)
        theta2 = np.arctan2(x, np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2)) - np.arctan2(self.l3 * np.sin(theta3),
                                                                                     self.l2 + self.l3 * np.cos(theta3))
        return [-theta1, theta2, theta3]

    def IK_L(self, x, y, z):
        D = (x ** 2 + y ** 2 + z ** 2 - self.l1 ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)

        theta3 = np.arctan2(-np.sqrt(1 - D ** 2), D)
        theta1 = -np.arctan2(-z, y) - np.arctan2(np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2), -self.l1)
        theta2 = np.arctan2(x, np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2)) - np.arctan2(self.l3 * np.sin(theta3),
                                                                                     self.l2 + self.l3 * np.cos(theta3))
        return [theta1, theta2, theta3]


    def IK_R_2(self, x, y, z):
        D = (x ** 2 + y ** 2 + z ** 2 - self.l1 ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        theta3 = np.arctan2(-np.sqrt(1 - D ** 2), D)
        theta1 = -np.arctan2(z, y) - np.arctan2(np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2), -self.l1)
        theta2 = np.arctan2(-x, np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2)) - np.arctan2(self.l3 * np.sin(theta3),
                                                                                     self.l2 + self.l3 * np.cos(theta3))
        return [-theta1, theta2, theta3]

    def IK_L_2(self, x, y, z):
        D = (x ** 2 + y ** 2 + z ** 2 - self.l1 ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)

        theta3 = np.arctan2(-np.sqrt(1 - D ** 2), D)
        theta1 = -np.arctan2(z, -y) - np.arctan2(np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2), -self.l1)
        theta2 = np.arctan2(-x, np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2)) - np.arctan2(self.l3 * np.sin(theta3),
                                                                                     self.l2 + self.l3 * np.cos(theta3))
        return [theta1, theta2, theta3]


    def IK_2D(self, x, z):
        # default condition l2 = l3
        x = -x
        cos_theta3 = (x ** 2 + z ** 2 - (self.l2 ** 2 + self.l3 ** 2)) / (2 * self.l2 * self.l3)
        theta3_rad = np.arccos(cos_theta3)
        alpha = theta3_rad / 2
        theta2_rad = np.pi / 2 - theta3_rad / 2 - np.arctan2(x, z)
        return -theta2_rad, theta3_rad


    def positions_control2(self, body_name, LF, RF, LB, RB):
        LF = np.array(LF)
        RF = np.array(RF)
        LB = np.array(LB)
        RB = np.array(RB)
        LF, RF, LB, RB = -LF, -RF, -LB, -RB

        # calibration for the urdf error
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=2,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LF[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force, maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=3,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LF[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=5,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LF[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=7,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RF[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=8,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RF[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=10,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RF[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=12,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LB[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=13,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LB[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=15,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LB[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=17,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RB[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=18,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RB[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=20,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RB[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

    def table_control(self,bodyname,angle1,angle2,angle3):
        p.setJointMotorControl2(bodyUniqueId=bodyname,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle1,
                            force=100,
                            positionGain=0.2,
                            velocityGain= 0.2
                            )
        p.setJointMotorControl2(bodyUniqueId=bodyname,
                            jointIndex=4,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle2,
                            force=1000,
                            positionGain=0.2,
                            velocityGain= 0.2,
                    )
        p.setJointMotorControl2(bodyUniqueId=bodyname,
                            jointIndex=10,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle3,
                            force=1000,
                            positionGain=0.2,
                            velocityGain= 0.2,
            )



if __name__ == '__main__':
    leg_controller = Leg()
    angle = leg_controller.IK_L_2(0.0113,0.045,-0.165)
    print(f'angle =={angle}')