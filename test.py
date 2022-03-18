import gym
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os

'''
This is my (Nathan Smith) implementation of Q-Learning for OpenAI's gym CartPole problem.
'''

def Q_Learning():

    env = gym.make('Ant-v3')
    done = False

    env.reset()
    while True:
        env.render()
        next_observation,reward,done,_ = env.step(env.action_space.sample())
        #print(env.action_space)
        print(reward)
        print(done)
    #env.close()
    print("New simulation")

    '''
    actions = [0,1] # Action space. 0 - Pushes car to the left, 1 - Pushes cart to the right

    bc_x,bc_x_dot,bc_theta,bc_theta_dot = 1,1,6,12 # Number of bins per state value (Bin counts)
    
    # Creating bins
    x_bins = np.linspace(-4.8,4.8,bc_x-1)
    x_dot_bins = np.linspace(-0.5,0.5,bc_x_dot-1) # -0.5 to 0.5 was chosen simply through observing the cart. It doesn't seem to go beyond these values often.
    theta_bins = np.linspace(-0.418,0.418,bc_theta-1)
    theta_dot_bins = np.linspace(-1,1,bc_theta_dot-1) # Same story here. 
    bins = (x_bins,x_dot_bins,theta_bins,theta_dot_bins)

    observation = env.reset()
    # Discretize the data observation data
    #state = tuple([np.digitize(observation[i],bins[i]) for i in range(len(observation))])

    # Parameters (You may change these)
    gamma = 1

    alpha = 1 # The starting value of alpha
    min_alpha = 0.1 # The minimum value alpha will go to
    alpha_decay = 0.00008

    epsilon = 1
    min_epsilon = 0.1
    epsilon_decay = 0.00008

    episode_count = 1000 # Max episode count
    solved_reward = 150 # The average reward threshold we wish to get to

    # The Q-Table of values. e.g. Q[S][A]
    Q_table = {}

    # Initialize all Q(s,a) to 0
    for x in range(bc_x):
        for x_dot in range(bc_x_dot):
            for theta in range(bc_theta):
                for theta_dot in range(bc_theta_dot):
                    Q_table[(x,x_dot,theta,theta_dot)] = {}
                    for a in actions:
                        Q_table[(x,x_dot,theta,theta_dot)][a] = 0
        
    episode = 0 # Episode number we're on
    time_steps = 0 # Total time steps
    streak = 0 # Number of times the reward threshold has been reached in a row
    previous_total_rewards = [] # History of reward per episode

    while episode < episode_count: 
        alpha = max(min_alpha,alpha/(1+alpha_decay*episode))
        epsilon = max(min_epsilon,epsilon/(1+epsilon_decay*episode))

        observation = env.reset() # x x_dot theta theta_dot (Initialize S)
        # Discretize the observation data
        #tate = tuple([np.digitize(observation[i],bins[i]) for i in range(len(observation))])

        done = False
        total_reward = 0
        while not done: # Loop for each step of episode
            time_steps += 1
            env.render() # Uncomment this to watch it learn! Dramatically increases computation time.

            # Choose A from S using policy derived from Q (e.g. epsilon-greedy)
            if np.random.random() < epsilon:
                action = np.random.choice(actions)
            else:
                action = max(Q_table[state],key=Q_table[state].get)

            # Take action A, observe R, S'
            next_observation,reward,done,_ = env.step(action)
            
            total_reward += reward

            # Discretize the data
            next_state = tuple([np.digitize(next_observation[i],bins[i]) for i in range(len(next_observation))])

            Q_table[state][action] = Q_table[state][action]+alpha*(reward + gamma*max(Q_table[next_state].values()) - Q_table[state][action])
            state = next_state # S <- S'

            """ This section of code is for dev purposes. It significantly increases computation time. Remove/add '#' to quote/unquote it out
            if episode != 0:
                os.system("clear")
                print("State: ",state)
                print("Alpha: ",alpha)
                print("Epsilon: ",epsilon)
                print("Streak: ",streak)
                print("Episode: {}\t Total reward: {}".format(episode,total_reward))
                print("Avg over last {} episodes: {:.2f}".format(len(previous_total_rewards[-100:]),np.mean(previous_total_rewards[-100:])))
            """

        if total_reward >= solved_reward:
            streak += 1
        else:
            streak = 0

        previous_total_rewards.append(total_reward)
        episode += 1

        # If the algorithm solved the cartpole problem, stop the simulation.
        if len(previous_total_rewards) >= 100 and np.mean(previous_total_rewards[-100:]) > solved_reward:
            print()
            print("Avg over last {} episodes: {:.2f}".format(len(previous_total_rewards[-100:]),np.mean(previous_total_rewards[-100:])))
            print("Solved in {} episodes".format(episode))
            break

    #plt.plot(previous_total_rewards)
    #plt.show()
    return None
    '''


if __name__ == "__main__":
    Q_Learning()