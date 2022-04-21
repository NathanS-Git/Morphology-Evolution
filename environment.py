import os
import copy
from gym import utils
from gym.envs.mujoco import mujoco_env
from gym.envs.mujoco.ant_v3 import AntEnv
from gym.wrappers.order_enforcing import OrderEnforcing
from gym.wrappers.time_limit import TimeLimit


def make(morphology_name):
    """Instantiates an instance of the environment"""

    env = Env(morphology_name)
    env = OrderEnforcing(env)
    env = TimeLimit(env, max_episode_steps=1000)
    
    return env


class Env(AntEnv):
    """ Wrapper for a modified OpenAI Gym to run with custom morphology.
    This also allows us to easily replace this with a unique implementation not relying on
    OpenAI Gym.  """

    def __init__(
        self,
        xml_file,
        
        # Simulation Perameters
        ctrl_cost_weight=0.05,
        contact_cost_weight=5e-4,
        healthy_reward=0.0,
        terminate_when_unhealthy=True,
        healthy_z_range=(0.0, 2.0),
        contact_force_range=(-1.0, 1.0),
        reset_noise_scale=0.1,
        exclude_current_positions_from_observation=True,
    ):

        utils.EzPickle.__init__(**locals())

        self._ctrl_cost_weight = ctrl_cost_weight
        self._contact_cost_weight = contact_cost_weight

        self._healthy_reward = healthy_reward
        self._terminate_when_unhealthy = terminate_when_unhealthy
        self._healthy_z_range = healthy_z_range

        self._contact_force_range = contact_force_range

        self._reset_noise_scale = reset_noise_scale

        self._exclude_current_positions_from_observation = (
            exclude_current_positions_from_observation
        )

        pwd = os.getcwd()

        mujoco_env.MujocoEnv.__init__(self, pwd+'/'+xml_file, 5)
