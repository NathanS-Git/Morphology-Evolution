import numpy as np
import mujoco_py
import torch
import time
import gym
import os
import glob
import re

import TD3
import utils

from MorphologyGeneration import MorphologyGeneration as morphgen

# Simulation defaults
CTRL_COST_WEIGHT =          0.05
CONTACT_COST_WEIGHT =       5e-4
HEALTHY_REWARD =            0
TERMINATE_WHEN_UNHEALTHY =  True
HEALTHY_Z_RANGE =           (0, 2)
CONTACT_FORCE_RANGE =       (-1, 1)
RESET_NOISE_SCALE =         0.1
EXCLUDE_CURRENT_POSITIONS_FROM_OBSERVATION = True


def eval_policy(policy, seed, morphology_file, eval_episodes=10):
    pwd = os.getcwd()

    eval_env = gym.make("Ant-v3",xml_file=f"{pwd}/{morphology_file}",
        ctrl_cost_weight=CTRL_COST_WEIGHT,
        contact_cost_weight=CONTACT_COST_WEIGHT,
        healthy_reward=HEALTHY_REWARD,
        terminate_when_unhealthy=TERMINATE_WHEN_UNHEALTHY,
        healthy_z_range=HEALTHY_Z_RANGE,
        contact_force_range=CONTACT_FORCE_RANGE,
        reset_noise_scale=RESET_NOISE_SCALE,
        exclude_current_positions_from_observation=EXCLUDE_CURRENT_POSITIONS_FROM_OBSERVATION
    )
    eval_env.seed(seed + 100)

    avg_reward = 0.
    for _ in range(eval_episodes):
        state, done = eval_env.reset(), False
        while not done:
            action = policy.select_action(np.array(state))
            state, reward, done, _ = eval_env.step(action)
            avg_reward += reward

    avg_reward /= eval_episodes

    print("---------------------------------------")
    print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    print("---------------------------------------")
    return avg_reward


def eval_morphology(morphology_file,episode_count=1e7):

    file_name = morphology_file
    
    morphology_name = re.search('[^/.]*(?=.xml)', file_name).group(0)

    print("---------------------------------------")
    print(f"Policy: TD3, Env: {morphology_name}, Seed: 0")
    print("---------------------------------------")

    if not os.path.exists("./results/{gen}"):
        os.makedirs("./results")

    if not os.path.exists("./models"):
        os.makedirs("./models")

    pwd = os.getcwd()

    env = gym.make("Ant-v3",xml_file=f"{pwd}/{morphology_file}",
        ctrl_cost_weight=CTRL_COST_WEIGHT,
        contact_cost_weight=CONTACT_COST_WEIGHT,
        healthy_reward=HEALTHY_REWARD,
        terminate_when_unhealthy=TERMINATE_WHEN_UNHEALTHY,
        healthy_z_range=HEALTHY_Z_RANGE,
        contact_force_range=CONTACT_FORCE_RANGE,
        reset_noise_scale=RESET_NOISE_SCALE,
        exclude_current_positions_from_observation=EXCLUDE_CURRENT_POSITIONS_FROM_OBSERVATION
    )

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

    evaluations = []
    try: # Attempt to load trained files if they exist
        policy.load(f"./models/{morphology_name}")

        assert os.path.exists(f"./results/{morphology_name}.npy"), "Could load model, but previous data did not exist."

        previous_data = np.load(f"./results/{morphology_name}.npy")
        evaluations = list(np.append(previous_data,evaluations))
    except FileNotFoundError:
        # Evaluate untrained policy
        evaluations = [eval_policy(policy, 0, morphology_file)]

    replay_buffer = utils.ReplayBuffer(state_dim, action_dim)

    starting_episode = len(evaluations-1)*5e3

    state, done = env.reset(), False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    for t in range(starting_episode,int(episode_count)):

        episode_timesteps += 1

        # Select action randomly or according to policy
        if t < 25e3:
            action = env.action_space.sample()
        else:
            action = (
                policy.select_action(np.array(state))
                + np.random.normal(0, max_action * 0.1, size=action_dim)).clip(-max_action, max_action)

        # Perform action
        next_state, reward, done, _ = env.step(action) 
        done_bool = float(done) if episode_timesteps < env._max_episode_steps else 0

        # Store data in replay buffer
        replay_buffer.add(state, action, next_state, reward, done_bool)

        state = next_state
        episode_reward += reward

        # Train agent after collecting sufficient data
        if t >= 25e3:
            policy.train(replay_buffer, 256)

        if done: 
            # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
            # Reset environment
            state, done = env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1 

        # Evaluate episode
        if (t + 1) % 5e3 == 0:
            evaluations.append(eval_policy(policy, 0, morphology_file))
            np.save(f"./results/{morphology_name}", evaluations)
            policy.save(f"./models/{morphology_name}")

    return max(evaluations)


if (__name__ == "__main__"):
    xml_files = glob.glob("./morphology/*.xml")
    xml_files.sort()

    file_name = xml_files[-1] # By default, use the latest morphology
    file_name = "./morphology/Ant_Mod_0.xml"

    eval_morphology(file_name)