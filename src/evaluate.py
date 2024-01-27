import numpy as np
import torch
import os
import glob
import re
import wandb

import TD3
import utils
import environment

def eval_policy(policy, morphology_file, eval_episodes=10):
    """ Evaluates the currently trained policy values and returns the average reward. """

    eval_env = environment.make(morphology_file)
    eval_env.seed(100)

    avg_reward = 0.
    for _ in range(eval_episodes):
        state, done = eval_env.reset(), False
        while not done:
            action = policy.select_action(np.array(state))
            state, reward, done, _ = eval_env.step(action)
            avg_reward += reward

    avg_reward /= eval_episodes

    #print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    return avg_reward


def eval_morphology(morphology_file,episode_count=1e7,gen=1):
    """ Trains a morphology given its xml file location and return
    its highest fitness achieved after training. """

    file_name = morphology_file
    
    morphology_name = re.search('[^/.]*(?=.xml)', file_name).group(0)

    print("--------------------------------------------------------------")
    print(f"Policy: TD3, Env: {morphology_name}, Seed: 0")
    print("--------------------------------------------------------------")

    if not os.path.exists("./results"):
        os.makedirs("./results")

    if not os.path.exists("./models"):
        os.makedirs("./models")

    try:
        env = environment.make(file_name)
    except AttributeError:
        print("{} CONTAINS NO LEGS".format(file_name))
        # Give it negative infinity fitness
        return float('-inf')

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
        evaluations = [eval_policy(policy, morphology_file)]

    replay_buffer = utils.ReplayBuffer(state_dim, action_dim)

    starting_episode = (len(evaluations)-1)*5000

    state, done = env.reset(), False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    wandb.init(
        project="Morphology Evolution",
        name=morphology_file,
        config=kwargs
    )

    for t in range(starting_episode,int(episode_count)):

        episode_timesteps += 1

        # Select action randomly or according to policy
        if t < 25e3:
            action = env.action_space.sample()
        else:
            action = (policy.select_action(np.array(state))
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
            if (episode_num+1) % 10 == 0:
                # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
                print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
            # Reset environment
            state, done = env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1 

        # Evaluate episode
        if (t + 1) % 5e3 == 0:
            eval = eval_policy(policy, morphology_file)
            wandb.log({f"Generation {gen:02} (fitness per 5k episodes)": eval})
            evaluations.append(eval)
            np.save(f"./results/{morphology_name}", evaluations)
            policy.save(f"./models/{morphology_name}")

    return max(evaluations)


if (__name__ == "__main__"):
    xml_files = glob.glob("./morphology/*.xml")
    xml_files.sort()

    file_name = xml_files[-1] # By default, use the latest morphology
    file_name = "./morphology/Morphology_2022-Mar-28 10:58:09 Gen:2 0.xml"

    eval_morphology(file_name)
