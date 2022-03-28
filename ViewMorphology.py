import numpy as np
import mujoco_py
import torch
import time
import os
import glob
import re

import TD3
import utils
import environment

from MorphologyGeneration import MorphologyGeneration as morphgen


def view(morphology_file):

    file_name = morphology_file
    morphology_name = re.search('[^/.]*(?=.xml)', file_name).group(0)

    print("Viewing morphology: {}".format(morphology_name))

    pwd = os.getcwd()
    
    # Make sure these values are the same as evaluation
    env = environment.make(file_name)

    # Set seeds
    env.seed(0)
    env.action_space.seed(0)
    torch.manual_seed(0)
    np.random.seed(0)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0] 
    max_action = float(env.action_space.high[0])

    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": 0.99,
        "tau": 0.005,
        "policy_noise": 0.2 * max_action,
        "noise_clip": 0.5 * max_action,
        "policy_freq": 2
    }

    policy = TD3.TD3(**kwargs)

    try:
        policy_file = morphology_name
        policy.load(f"./models/{policy_file}")
        policy_loaded = True
    except:
        print("Failed to load pre-trained model. Defaulting to random values")
        policy_loaded = False

    episodes = 10000000

    if policy_loaded:
        avg_reward = 0.
        for _ in range(episodes):
            state, done = env.reset(), False
            cum_reward = 0
            ep = 0
            while not done:
                ep += 1
                action = policy.select_action(np.array(state))
                state, reward, done, _ = env.step(action)
                avg_reward += reward
                cum_reward += reward
                print("Episode: {:4}\tReward: {:.2f}".format(ep,cum_reward),end='\r')
                env.render()
                time.sleep(0.02)
            print()

        avg_reward /= episodes

    else:
        model = mujoco_py.load_model_from_path(file_name)
        sim = mujoco_py.MjSim(model)
        viewer = mujoco_py.MjViewer(sim)
        for _ in range(episodes):
            for motor in range(len(sim.data.ctrl)): 
                sim.data.ctrl[motor] = np.random.random()*2-1
            viewer.render()
            sim.step()


if (__name__ == "__main__"):    
    xml_files = glob.glob("./morphology/*.xml")
    xml_files.sort()

    file_name = xml_files[-1] # By default, use the latest morphology
    file_name = "./morphology/Ant_Mod_0.xml"

    view(file_name)