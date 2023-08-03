
# spot_gym_3 is used to training climb on a 10° degree
# action space is 4, with sin function, it's periodic optimal signal

from pybullet_utils import bullet_client
import pybullet as p
import pybullet_data as pd
import gym
from gym import spaces
import numpy as np
from mark_robot import Robot
from CPGenerator import CPG
from mark_leg import Leg
import time
from gym.utils import seeding

class Spot_gym(gym.Env):
    def __init__(self, render: bool = False, number_motor=12):
        self.render_flag = render
        self.spot_leg = Leg()
        if self.render_flag:
            self._pybullet_client = bullet_client.BulletClient(connection_mode=p.GUI)
        else:
            self._pybullet_client = bullet_client.BulletClient()

        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self._pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.robot = self._pybullet_client.loadURDF("../urdf/rex.urdf",[0,0,0.3],
                                                   baseOrientation = self._pybullet_client.getQuaternionFromEuler([0, 0,np.pi]),
                                                   #flags=self._pybullet_client.URDF_USE_INERTIA_FROM_FILE,
                                                   # useFixedBase = 1
                                                   )
        self.planeID = self._pybullet_client.loadURDF("plane.urdf")
        # self.slope = self._pybullet_client.loadURDF("../urdf/slope/slope.urdf",
        #                                            [2.5, 0, 0.0],
        #                                             # baseOrientation=self._pybullet_client.getQuaternionFromEuler(
        #                                             #     [np.pi/2, 0, 0]),
        #
        #                                             # flags=p.URDF_USE_INERTIA_FROM_FILE,
        #                                             useFixedBase=1
        #                                             )
        # self._pybullet_client.changeDynamics(self.slope, linkIndex=-1, lateralFriction=1.0)
        self.spot = Robot(self.robot, self._pybullet_client)
        self.action_bound = 1
        # action_space
        action_high = np.array([self.action_bound] * 4)
        self.action_space = spaces.Box(
            low=0, high=action_high,
            dtype=np.float32)
        # observation space
        observation_high = self.spot.get_observation_upper_bound()
        self.observation_space = spaces.Box(
            low=-observation_high, high=observation_high,
            dtype=np.float64)
        self.gait_generator = CPG(step_length=0.025,ground_clearance=0.025,ground_penetration=0.001,Tswing=0.3,Tstance=0.3,initial_x=-0.0)
        self.control_fre = 200
        self.dt = 1./self.control_fre # should be related to leg control frequency

        self.forward_weightX = 0.015
        self.forward_weightY = 0.02
        self.forwardV_weight = 0.01
        self.direction_weight = -0.001
        self.shake_weight = -0.005
        self.height_weight = -0.05
        self.joint_weight = -0.001

        self.angleFromReferen = np.array([0] * 12)

        self.pre_coorX  = 0.165
        self.pre_height = 0.194
        #
        # optimize signal
        self.opti_range_upper = np.deg2rad(8)
        self.opti_range_knee = np.deg2rad(6)
        self.referSignal = 1.
        self.optimSignal = 1.

        self.reward_details = np.array([0.] * 5, dtype=np.float32)
        self.reward_detail_dict = {'forwardX': 0, 'forwardY': 0, 'forwardV_reward': 0, 'shaking_reward': 0,
                                   'height_reward': 0}

        self.step_num = 0
        self.initial_count = 0
        self.train_steps = 0
        self.reset_terrain_flag = True
        # 0.05382755820485801, 0, 0.19330842049777203
        # self.terrain = Terrain("random").generate_terrain(self._pybullet_client,0.02)



    def reset(self):
        # ----------initialize pubullet env----------------
        self._pybullet_client.setGravity(0, 0, 0)
        self._pybullet_client.resetBasePositionAndOrientation(bodyUniqueId=self.robot, posObj=[0, 0, 0.3],
                                                            ornObj=self._pybullet_client.getQuaternionFromEuler([0, 0, np.pi] ) )
        self.spot = Robot(self.robot, self._pybullet_client)


        #  ----------------------------------initial parameter------------------#
        self.reward_details = np.array([0.] * 5, dtype=np.float32)
        self.step_num = 0
        self.pre_coorX  = 0.165
        self.pre_height = 0.194
        self.gait_generator.trot_timer = 0.
        #  ----------------------------------initial parameter------------------#

        while self.initial_count < 100:
            self._pybullet_client.setGravity(0, 0, 0)
            self.initial_count += 1
            self.spot_leg.positions_control2(self.robot, self.spot.stand_pose[0], self.spot.stand_pose[1],
                                                         self.spot.stand_pose[2], self.spot.stand_pose[3])
            self.spot.motor_angle = np.hstack((self.spot.stand_pose))
            self._pybullet_client.resetBasePositionAndOrientation(bodyUniqueId=self.robot, posObj=[0, 0, 0.3],
                                                                  ornObj=self._pybullet_client.getQuaternionFromEuler(
                                                                      [0, 0, np.pi]))

            self._pybullet_client.resetBaseVelocity(objectUniqueId=self.robot, linearVelocity=[0, 0, 0],
                                                    angularVelocity=[0, 0, 0])
            p.stepSimulation()


        while self.initial_count < 250:
            self._pybullet_client.setGravity(0, 0, -9.8)
            self.initial_count += 1
            self.spot_leg.positions_control2(self.robot, self.spot.stand_pose[0], self.spot.stand_pose[1],
                                                         self.spot.stand_pose[2], self.spot.stand_pose[3])
            self.spot.motor_angle = np.hstack((self.spot.stand_pose))
            p.stepSimulation()
        self.initial_count = 0
        return self.get_observation(0)


    def get_observation(self,gait_timer):
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in ")
        observation = self.spot.get_observation(gait_timer)
        # print(f"observation_space === { observation }")


        return observation

    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def close(self):
        if self.physics_client_id >= 0:
            self._pybullet_client.disconnect()
        self.physics_client_id = -1


    def merge_action(self, action):
        LF = [0, 0, 0]
        RF = [0, 0, 0]
        LB = [0, 0, 0]
        RB = [0, 0, 0]

        # shoulder optimize signal from -3° to 3 °
        LF [1] = RB [1] = action[0] * self.opti_range_upper * np.sin(self.gait_generator.trot_timer)
        LF [2] = RB [2] = action[1] * self.opti_range_knee * -np.sin(self.gait_generator.trot_timer)
        RF [1] = LB [1] = action[2] * self.opti_range_upper * np.sin(self.gait_generator.trot_timer)
        RF [2] = LB [2] = action[3] * self.opti_range_knee * -np.sin(self.gait_generator.trot_timer)
        # hip,knee optimize signal from -15° to 15 °
        return np.hstack((LF, RF, LB, RB)) * self.optimSignal + self.angleFromReferen * self.referSignal


    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")


        if self.step_num >= 0 and self.step_num <= 20:
            random_force = np.random.uniform(-3, 3, 3)
            self._pybullet_client.applyExternalForce(objectUniqueId=self.robot, linkIndex=-1,
                                                     forceObj=[random_force[0], random_force[1], random_force[2]],
                                                     posObj=[0.165, 0, 0.194],
                                                     flags=self._pybullet_client.WORLD_FRAME)

        angles = self.gait_generator.trot_generator(self.gait_generator.trot_timer, 'straight', 0.8)  # degree
        angles = angles * np.pi / 180.    # radius
        self.gait_generator.trot_timer += self.dt
        self.angleFromReferen = angles

        action_on_motor = self.merge_action(action)
        self.spot_leg.positions_control2(self.robot, action_on_motor[0:3], action_on_motor[3:6],
                                          action_on_motor[6:9], action_on_motor[9:12])
        self.spot.motor_angle = action_on_motor


    def reward_function(self,reward_items):
        x_coor  = reward_items[0]
        y_coor  = reward_items[1]
        linearX = reward_items[2]
        linearY, linearZ = reward_items[3:5]
        Wx, Wy, Wz   = reward_items[5:8]
        roll , pitch ,yaw = reward_items[8:11]
        height       = reward_items[11]

        forwardX_reward = self.forward_weightX * (x_coor - self.pre_coorX)
        forwardY_reward = -self.forward_weightY * np.abs(y_coor)
        forwardV_reward = self.forwardV_weight * linearX / 4
        shaking_reward = self.shake_weight *     ( np.exp( -1/(Wx**2+Wy**2+Wz**2+ 1e-10)))/5+ \
                         self.shake_weight* ( roll**2 + pitch**2 + yaw**2)
        height_reward = 0

        reward_details = np.array(
            [forwardX_reward, forwardY_reward, forwardV_reward, shaking_reward, height_reward])
        self.reward_details += reward_details
        reward = np.sum(reward_details)


        return  reward



    def step(self, action):

        self.apply_action(action)
        self._pybullet_client.stepSimulation()
        self.step_num += 1
        self.train_steps+=1
        state = self.get_observation(0.)
        # print(f'self.gait_generator.trot_timer=={self.gait_generator.trot_timer}')
        reward_items = self.spot.get_reward_items()


        reward = self.reward_function(reward_items)
        roll, pitch, yaw = self.spot.get_ori()
        x = self.spot.get_Global_Coor()[0]
        y = self.spot.get_Global_Coor()[1]


        # condition for stop
        xyz = self.spot.get_Global_Coor()
        # print(f'x,y,z===={xyz[0]}{xyz[1]}{xyz[2]}')
        # print(f'roll==={np.rad2deg(roll)},pitch=={np.rad2deg(pitch)},yaw==={np.rad2deg(yaw)}')

        if self.step_num > 2500:
            done = True
        elif np.abs(roll) > np.deg2rad(45) or np.abs(pitch) > np.deg2rad(45) or np.abs(yaw) > np.deg2rad(45) or y > 0.5 or x <-0.2:
            reward = -8
            done = True
        else:
            done = False
        info = {}
        return state, reward, done, info

    def return_reward_details(self):
        self.reward_detail_dict['forwardX'] = self.reward_details[0]
        self.reward_detail_dict['forwardY'] = self.reward_details[1]
        self.reward_detail_dict['forwardV_reward'] = self.reward_details[2]
        self.reward_detail_dict['shaking_reward'] = self.reward_details[3]
        self.reward_detail_dict['height_reward'] = self.reward_details[4]



    def test_no_RL(self,model, test_round,test_speed):
        done = False
        self.optimSignal = 0
        all_episode_reward = []
        max_height = []
        max_distance = []
        r_data = []
        y_data = []
        _wx = []
        _wy = []
        _wz = []

        for i in range(test_round):
            height_set = []
            distance_set = []

            obs = self.reset()
            episode_reward = 0
            while True:
                time.sleep(test_speed)
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                # check observation space
                # print(f'obs=={obs}')
                # check rpy
                # ori=self.spot.get_ori()
                # print(f'ori=={ori}')
                episode_reward += reward

                pose,ori = self._pybullet_client.getBasePositionAndOrientation(self.robot)
                height_set.append(pose[2])
                distance_set.append(pose[0])
                ori = self._pybullet_client.getEulerFromQuaternion(ori)

                r_data.append(ori[0])
                y_data.append(ori[1])
                _,wxyz = self._pybullet_client.getBaseVelocity(self.robot)
                _wx.append(wxyz[0])
                _wy.append(wxyz[1])
                _wz.append(wxyz[2])


                if done:
                    break
            all_episode_reward.append(episode_reward)
            height_set = np.array(height_set)
            distance_set = np.array(distance_set)

            max_height.append(np.max(height_set))
            max_distance.append(np.max(distance_set))

            #----------save_data-------#
            self.save_data("wx_sim.txt",_wx)
            self.save_data("wy_sim.txt",_wy)
            self.save_data("wz_sim.txt",_wz)
            #----------save_data-------#

            print(f'episode_reward=={episode_reward}')
            print("all_reward==={},average reward=={}".format(all_episode_reward,
                                                              np.sum(all_episode_reward) / test_round))
            self.return_reward_details()
            print(self.reward_detail_dict)
            all_episode_reward.append(episode_reward)
            print(f'all reward_episode is {all_episode_reward}')
            print(f"train_steps=={self.train_steps}")

            # print(f"max_height =={max_height}")
            # print(f"max_distance =={max_distance}")
            print(f"average_height =={np.average(max_height)}")
            print(f"average_distance =={np.average(max_distance)}")




        return all_episode_reward


    def test_model(self, model, test_round,test_speed, ):
        all_episode_reward = []

        max_height = []
        max_distance = []

        for i in range(test_round):
            height_set = []
            distance_set = []
            episode_reward = 0
            obs = self.reset()
            while True:
                time.sleep(test_speed)
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                episode_reward += reward

                pose,ori = self._pybullet_client.getBasePositionAndOrientation(self.robot)
                height_set.append(pose[2])
                distance_set.append(pose[0])

                if done:
                    break
            height_set = np.array(height_set)
            distance_set = np.array(distance_set)

            max_height.append(np.max(height_set))
            max_distance.append(np.max(distance_set))
            all_episode_reward.append(episode_reward)

            # print(f"max_height =={max_height}")
            # print(f"max_distance =={max_distance}")
            # print("episode reward===={}".format(episode_reward))
            self.return_reward_details()
            print(self.reward_detail_dict)
        all_episode_reward = np.array(all_episode_reward)
        print("all_reward==={},average reward=={}".format(all_episode_reward,np.sum(all_episode_reward)/test_round ))
        print(f"total_train_step =={self.train_steps}")

        # print(f"max_height =={max_height}")
        # print(f"max_distance =={max_distance}")


        print(f"average_height =={np.average(max_height)}")
        print(f"average_distance =={np.average(max_distance)}")


        return all_episode_reward


    def train(self,model,save_path,training_steps):
        model.learn(training_steps)
        model.save(save_path)

    def reset_terrain(self):
        self._pybullet_client.removeBody(self.terrain)
        # self._pybullet_client.changeVisualShape(self.terrain, -1, rgbaColor=[0.5, 0.5, 0, 0.5])
        # self._pybullet_client.loadTexture("texture/grass.png")
        self._pybullet_client.resetBasePositionAndOrientation(self.terrain, [0, 0, 0], [0, 0, 0, 1])

    def save_data(self,file_path,float_data):
        try:
            with open(file_path,'w') as file:
                for float_value in float_data:
                    float_string = "{:.2f}".format(float_value)
                    file.write(float_string+ "\n")
            print(f"float data has been saved successfully")
        except Exception as e:
            print(f"error==={e}")




if __name__ == '__main__':
    from  stable_baselines3 import PPO
    from  stable_baselines3.common.env_checker import check_env
    from  stable_baselines3.common.callbacks import CheckpointCallback



    env = Spot_gym(render=True)
    # # -----------------training---------------#
    # model = PPO(policy="MlpPolicy", env=env, verbose=1,batch_size=512,learning_rate= 3e-4)

    # print(model.policy)
    # t1 = time.time()
    # checkpoint_callback = CheckpointCallback(
    #     save_freq=2000000,
    #     save_path="result",
    #     name_prefix="train_result_6m_slope_PPO6",
    #     save_replay_buffer=True,
    #     save_vecnormalize=True,
    # )
    # model = PPO(policy="MlpPolicy", env=env, verbose=1,batch_size=512,tensorboard_log="./result/",learning_rate= 3e-4)
    # model.learn(6000000, callback=checkpoint_callback)
    # model.save('result/train_result_6m_slope_PPO6')
    # t2 = time.time()
    # print(t2-t1)
    # -----------------training---------------#

    #------------------test for no rl--------#
    model = PPO(policy="MlpPolicy", env=env, verbose=1, batch_size=512)
    env.test_no_RL(model,1,0.0)
    # ------------------test for no rl--------#

    # ----------------multi thread------------#

    #-----------------multi thread------------#

    # -----------------test---------------#
    # loaded_model = PPO.load('result/PPO4_result/train_result_10.5m_slope_PPO4_10500000_steps.zip')
    # env.test_model(loaded_model,20,0.00)
    # -----------------test---------------#


    # reward == 4.0863280547943726
    # average_height == 0.23634544199859275
    # average_distance == 0.74481987728033
    # 108099



# train_steps==99427
# average_height ==0.2492741744656458
# average_distance ==0.8190511115589811
# episode_reward==9.71940756446046
