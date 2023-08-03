# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2023/1/12 21:27

import pybullet as p
import pybullet_data as pd
from pybullet_utils import bullet_client
import numpy as np
import time
import spot_leg
class Spot:
    def __init__(self):
        self.pybullet_client = bullet_client.BulletClient(connection_mode = p.GUI)
        self.robot = self.pybullet_client.loadURDF("/home/xing/my_mini_spot/src/Pose/urdf/spot_old.urdf",[0,0,0.4],
                                                   useMaximalCoordinates=False,
                                                   flags=self.pybullet_client.URDF_USE_IMPLICIT_CYLINDER,
                                                   )
        self.counter = 0
        self.show_fre = 0.005
        self.leg_controller = spot_leg.Leg()
        self.stand_pose = [[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2],[0,np.pi/4,-np.pi/2]]
        self.sit_pose = [[0,0,0],[0,0,0],[0,np.pi/3,-np.pi*2/3],[0,np.pi/3,-np.pi*2/3]]
        self.liedown_pose = [[np.pi/3, np.pi/4,-np.pi*2/3 ],[-np.pi/3, np.pi/4,-np.pi*2/3 ],[np.pi/3, np.pi/4,-np.pi*2/3],[-np.pi/3, np.pi/4,-np.pi*2/3 ]]

    def run(self):
        self.pybullet_client.resetSimulation()
        self.pybullet_client.setGravity(0, 0, 0)
        self.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = self.pybullet_client.loadURDF("plane.urdf")
        # self.planeID = self.pybullet_client.loadURDF("urdf/plane/plane_implicit.urdf")

        self.robot = self.pybullet_client.loadURDF("/home/xing/my_mini_spot/src/Pose/urdf/spot_old.urdf", [0, 0, 0.4],
                                                   useMaximalCoordinates=False,
                                                   flags=p.URDF_USE_IMPLICIT_CYLINDER,
                                                   baseOrientation = self.pybullet_client.getQuaternionFromEuler([0, 0, np.pi]),
                                                   # useFixedBase = 1

                                                   )
        num = p.getNumJoints(self.robot)
        print(f'num=={num}')

        posi_list = []
        color_list = []

        while True:
            if self.counter <200:
                # for i in range(30):
                #     link_info = p.getLinkState(self.robot,i)
                #     print(f'{i}link_info{link_info[0]}')
                p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

                time.sleep(self.show_fre)
                self.leg_controller.positions_control2(self.robot,
                                                   self.stand_pose[0],
                                                   self.stand_pose[1],
                                                   self.stand_pose[2],
                                                   self.stand_pose[3],
                                                   )

                # pre_position = p.getLinkState(self.robot, 6)[0]
                p.stepSimulation()

            elif self.counter<1000:

                p.setGravity(0,0,-9.8)
                p.stepSimulation()
                time.sleep(self.show_fre)

            elif self.counter<1500:
                time.sleep(self.show_fre*5)
                self.leg_controller.positions_control2(self.robot,
                                                       self.liedown_pose[0],
                                                       self.liedown_pose[1],
                                                       self.liedown_pose[2],
                                                       self.liedown_pose[3],
                                                       )

                # pre_position = p.getLinkState(self.robot, 6)[0]
                p.stepSimulation()
            elif self.counter<2000:
                time.sleep(self.show_fre*5)
                self.leg_controller.positions_control2(self.robot,
                                                       self.stand_pose[0],
                                                       self.stand_pose[1],
                                                       self.stand_pose[2],
                                                       self.stand_pose[3],
                                                       )

                # pre_position = p.getLinkState(self.robot, 6)[0]
                p.stepSimulation()


            self.counter += 1


if __name__ == '__main__':
    spot = Spot()
    spot.run()