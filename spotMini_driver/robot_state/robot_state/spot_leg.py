# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2023/1/12 22:31

import numpy as np



class Leg():
    def __init__(self,  shoulder2hip=0.055, hip2knee=0.108, knee2end=0.135):
        self.l1 = shoulder2hip
        self.l2 = hip2knee
        self.l3 = knee2end
        self.Position_Gain = 1
        self.Velocity_Gain = .5
        self.force = 3
        self.Max_velocity = 5
        #                    LF      RF     LB    RB
        self.joint_angle = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.t1 = 0.
        self.t2 = -0.5

    def time_reset(self):
        self.t1 = 0.0
        self.t2 = -0.5

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

if __name__ == '__main__':
    leg_controller = Leg()
    angle = leg_controller.IK_L_2(0.0113,0.045,-0.165)
    print(f'angle =={angle}')
    leg_controller = Leg()
    leg_controller = Leg()
    # leg_controller = Leg()
