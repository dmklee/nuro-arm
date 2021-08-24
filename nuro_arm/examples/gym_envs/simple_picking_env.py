import numpy as np
import pybullet as pb
import cv2
import gym

from nuro_arm import RobotArm, Camera, Cube


class ImageBasedPickingEnv(gym.Env):
    def __init__(self, mode='sim', seed=None):
        self.seed(seed)

        self.img_size = 80
        self.observation_space = gym.spaces.Box(0, 255, shape=(80,80),
                                                dtype=np.uint8)

        self.min_lifted_height = 0.1
        self.z_height = 0.0125
        self.workspace = np.array(((0.1, -0.05, self.z_height),
                                   (0.25, 0.05, self.z_height+1e-4)))

        self.robot = RobotArm(mode)
        self.camera = Camera(mode)
        self.camera.cap.stop_async()
        self.cube = Cube([0,0,self.z_height])

        self.action_space = gym.spaces.Box(self.workspace[0,:2],
                                           self.workspace[1,:2])

    def reset(self):
        self.robot.home()

        random_pos = np.random.uniform(*self.workspace)
        random_quat = pb.getQuaternionFromEuler((0, 0, np.random.uniform(0,np.pi)))
        self.cube.reset_pose(random_pos, random_quat)

        return self.get_obs()

    def step(self, a):
        assert self.action_space.contains(a)

        xyz = np.array((a[0], a[1], self.z_height))

        self.robot.set_gripper_state(0.5)
        self.robot.move_hand_to_state(xyz)
        self.robot.close_gripper()
        self.robot.home()

        obs = self.get_obs()

        cube_position = self.cube.get_position()
        reward = cube_position[2] > self.min_lifted_height
        if reward:
            done = True
        else:
            # end epsiode if cube is outside picking area
            done = not self.action_space.contains(cube_position[:2])
        info = {}

        return obs, reward, done, info

    def get_obs(self):
        img = self.camera.get_image()
        gray_img = np.mean(img, axis=2)

        gray_img = cv2.resize(gray_img[30:210, 80:260],
                              (self.img_size, self.img_size))

        return gray_img

env = ImageBasedPickingEnv()
obs = env.reset()
import matplotlib.pyplot as plt
plt.figure()
plt.imshow(obs)
plt.show()

