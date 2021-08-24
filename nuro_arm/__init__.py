from nuro_arm.robot.robot_arm import RobotArm
from nuro_arm.camera.camera import Camera
from nuro_arm.cube import Cube

from gym.envs.registration import register

register(
    id='nuroarm-simple-picking-v0',
    entry_point='nuro_arm.examples.gym_envs.simple_picking_env:ImageBasedPickingEnv',
    max_episode_steps=5
)
