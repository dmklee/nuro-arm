import numpy as np
import pybullet as pb
import cv2
import gym

from nuro_arm import RobotArm, Camera, Cube


class ImageBasedPickingEnv(gym.Env):
    def __init__(self, mode='sim', seed=None):
        """OpenAI gym environment where the xArm must grasp the block which is located randomly in the workspace.

        Observation Space: An 80x80 grayscale image (each pixel is an unsigned int between 0 and 255)
        Action Space: (x, y) position to grasp (x and y are both between 0 and 1)
        Reward: 1 if the robot successfully grasps the block. Otherwise, 0
        """
        self.seed(seed)

        self.img_size = 80
        self.observation_space = gym.spaces.Box(0, 255, shape=(1, 80, 80), dtype=np.uint8)

        self.min_lifted_height = 0.1
        self.z_height = 0.0125
        self.workspace = np.array(((0.1, -0.05, self.z_height),
                                   (0.25, 0.05, self.z_height+1e-4)))

        self.robot = RobotArm(mode, realtime=False)
        self.camera = Camera(mode)
        self.camera.cap.stop_async()
        self.cube = Cube([0, 0, self.z_height])

        self.action_space = gym.spaces.Box(low=0, high=1, shape=(2,))

    def reset(self):
        self.robot.home()

        random_pos = np.random.uniform(*self.workspace)
        random_quat = pb.getQuaternionFromEuler((0, 0, np.random.uniform(0,np.pi)))
        self.cube.reset_pose(random_pos, random_quat)

        return self.get_obs()

    def step(self, a):
        """Perform one step of the environment
        Arguments:
            a: (np.ndarray) The action consisting of x and y, both between 0 and 1
        Reward Condition:
            Reward = 1 when the robot successfully grasps the block
            Reward = 0 otherwise
        Returns:
            obs: the image observation taken by the camera
            reward: the scalar reward earned by the agent
            done: True when either the agent is successful or pushes the block off the workspace
            info: dict()
        """
        assert self.action_space.contains(a)

        x = self.workspace[0, 0] + a[0] * (self.workspace[1, 0] - self.workspace[0, 0])
        y = self.workspace[0, 1] + a[1] * (self.workspace[1, 1] - self.workspace[0, 1])

        xyz = np.array((x, y, self.z_height))

        self.robot.open_gripper()
        self.robot.move_hand_to(xyz)
        self.robot.close_gripper()
        self.robot.home()

        obs = self.get_obs()

        cube_position = np.array(self.cube.get_position())
        reward = cube_position[2] > self.min_lifted_height
        if reward:
            done = True
        else:
            # end epsiode if cube is outside picking area
            done = np.bitwise_and(cube_position[:2] < self.workspace[0,:2],
                                  cube_position[:2] > self.workspace[1,:2]).any()
        info = {}

        return obs, reward, done, info

    def get_obs(self):
        """Scales and Crops the observation image"""
        img = self.camera.get_image()
        gray_img = np.mean(img, axis=2)

        gray_img = cv2.resize(gray_img[30:210, 80:260],
                              (self.img_size, self.img_size))

        return gray_img


if __name__ == '__main__':
    env = ImageBasedPickingEnv()
    obs = env.reset()
    import matplotlib.pyplot as plt
    plt.figure()
    plt.imshow(obs)
    plt.show()
