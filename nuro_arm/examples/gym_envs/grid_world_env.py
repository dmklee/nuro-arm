import numpy as np
import gym
from nuro_arm import RobotArm


class GridWorldEnv(gym.Env):
    def __init__(self,
                 mode='sim',
                 n_grids=10,
                 seed=None,
                 ):
        self.seed(seed)

        self.n_grids = n_grids
        self.observation_space = gym.spaces.Box(0, n_grids, (2,), dtype=int)

        self.robot = RobotArm(mode)

        x_range = np.linspace(0.1, 0.3, self.n_grids)
        y_range = np.linspace(-0.1, 0.1, self.n_grids)
        z_val = 0.05
        self.grid_positions = np.dstack(np.meshgrid(x_range, y_range, [z_val]))

        # move in cardinal directions and noop
        self.action_deltas = ((0, 0), (1, 0), (0, 1), (-1, 0), (0, 1))
        self.action_space = gym.spaces.Discrete(5)

        self.goal = np.array((self.n_grids-1, self.n_grids-1))

    def reset(self):
        self.state = np.array((1, 1))

        return self.get_obs()

    def step(self, a):
        assert self.action_space.contains(a)

        new_state = np.add(self.state, self.action_deltas[a])
        self.state = np.clip(new_state, 0, self.n_grids-1)

        self.move_hand_to_state(self.state)

        obs = self.get_obs()
        reward = (self.state == self.goal).all()
        done = reward
        info = {}

        return obs, reward, done, info

    def get_obs(self):
        return self.state.copy()

    def move_hand_to_state(self, state):
        pos = self.grid_positions[state[0], state[1]]
        self.robot.move_hand_to(pos)

