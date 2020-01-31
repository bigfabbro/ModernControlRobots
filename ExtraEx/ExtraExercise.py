import time

import gym
import numpy as np

if __name__ == "__main__":

    env = gym.make('Pendulum-v0')
    env.reset()
    print("Actions: {} {}".format(env.action_space.low,
                                  env.action_space.high))
    # represents alpha, beta, gamma and delta
    weights = [1, 1, 1, 0]
    # observations three plus one (delta constant coefficient)
    observations = [0, 0, 0, 1]
    action = env.action_space.sample()
    accreward = 0
    for _ in range(1000):
        env.render()
        obs, reward, done, info = env.step(action)
        observations[:len(obs)] = obs
        action = [np.dot(weights, observations)]
        print("Executed action {}, received observation {}, and reward {}".format(action, obs, reward))
        #time.sleep(1)
    env.close()
