#!/usr/bin/env python
from __future__ import print_function

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import math
import numpy as np
import pickle
import cv2
import sys, traceback

from monodrive import Configuration, Simulator, SimulatorConfiguration, VehicleConfiguration
from monodrive.vehicles import SimpleVehicle


class SimpleNeuralNetwork(object):
    def __init__(self, hidden_layers, outputs, dimensionality,
                 resume = False, gamma = 0.99, down_sample=4,
                 batch_size=20, decay_rate=0.99, learning_rate=1e-4):
        self.hidden_layers = hidden_layers
        self.dimensionality = dimensionality
        self.gamma = gamma
        self.down_sample = down_sample
        self.batch_size = batch_size
        self.decay_rate = decay_rate
        self.learning_rate = learning_rate
        self.model = None
        if resume:
            self.model = self.load_model(hidden_layers, dimensionality)

        if self.model is None:
            self.model = {}
            self.model['W1'] = np.random.randn(hidden_layers, dimensionality) / np.sqrt(dimensionality)
            self.model['W2'] = np.random.randn(outputs, hidden_layers) / np.sqrt(hidden_layers)

        self.outputs = outputs

    def reset(self):
        self.grad_buffer = {k: np.zeros_like(v) for k, v in
                            self.model.items()}  # update buffers that add up gradients over a batch
        self.rmsprop_cache = {k: np.zeros_like(v) for k, v in self.model.items()}  # rmsprop memory
        self.prev_x = np.zeros(self.dimensionality)  # used in computing the difference frame
        self.xs, self.hs, self.dlogps, self.drs = [], [], [], []
        self.running_reward = None
        self.reward_sum = 0
        self.episode_number = 0
        self.last_reward = 0
        self.last_pawn = None

    def load_model(self, H, D):
        model = None
        try:
            f = open('save.p', 'rb')
            model['W1'] = np.random.randn(H, D) / np.sqrt(D)
            model['W1'].put(np.arange(0, 50), pickle.load(f))
            model['W1'].put(np.arange(50, 100), pickle.load(f))
            model['W1'].put(np.arange(100, 150), pickle.load(f))
            model['W1'].put(np.arange(150, 200), pickle.load(f))
            model['W2'] = pickle.load(f)
        except:
            pass
        return model

    def get_action_egreedy(self, x):
        # forward the policy network and sample an action from the returned probability
        aprob, h = self.policy_forward(x)
        max_index = np.argmax(aprob)
        if np.random.uniform() < aprob[max_index]:
            action = np.random.randint(self.outputs)
        else:
            action = max_index
        # print "apod max = " + str(aprob[indexMax])
        return aprob, h, action

    def sigmoid(self, x):
        sout = np.zeros(self.outputs)
        for i in range(len(x)):
            sout[i] = 1.0 / (1.0 + np.exp(-x[i]))  # sigmoid "squashing" function to interval [0,1]
        return sout

    def policy_forward(self, x):
        h = np.dot(self.model['W1'], x)
        h[h < 0] = 0  # ReLU nonlinearity
        logp = np.dot(self.model['W2'], h)
        p = self.sigmoid(logp)
        return p, h

    def get_policy_backward(self, eph, epdlogp, episode_x):
        """ backward pass. (eph is array of intermediate hidden states) """
        dW2 = np.dot(epdlogp.T, eph)
        dh = np.dot(epdlogp, self.model['W2'])
        dh[eph <= 0] = 0  # backpro prelu
        dW1 = np.dot(dh.T, episode_x)
        return {'W1': dW1, 'W2': dW2}

    def prepro(self, I):
        """ prepro 210x160x3 uint8 frame into 6400 (80x80) 1D float vector """
        # _LOG.debug("type of I: %s", type(I))
        # I[I > 250] = 100
        # I[I < 5] = 10
        I = I[::self.down_sample, ::self.down_sample]
        return I.astype(np.float).ravel()

    def discount_rewards(self, r):
        """ take 1D float array of rewards and compute discounted reward """
        discounted_r = np.zeros_like(r)  # Return an array of zeros with the same shape and type as a given array
        running_add = 0
        for t in reversed(range(0, r.size)):
            # if r[t] != 0: running_add = 0 # reset the sum, since this was a game boundary (pong specific!)
            running_add = running_add * self.gamma + r[t]
            discounted_r[t] = running_add
        return discounted_r

    def reverse(self, r):
        return reversed(range(0, r.size))

    def get_difference_frame(self, cur_x, prev_x):
        # input to network is the difference between the current frame (cur_x) and the previous frame (prev_x)
        return cur_x - prev_x if prev_x is not None else np.zeros(self.dimensionality)

    def store_values(self, x, h, d_error, reward):
        # record various intermediates (needed later for backprop)
        self.xs.append(x)  # observation
        self.hs.append(h)  # hidden state
        # grad that encourages the action that was taken to be taken (see http://cs231n.github.io/neural-networks-2/#losses if confused)
        self.dlogps.append(d_error)  # theta*x - y
        self.drs.append(reward)  # record reward (has to be done after we call step() to get reward for previous action)

    def get_error(self, action, aprob):
        y = np.zeros(self.outputs)
        y[action] = 1
        return np.subtract(y, aprob)

    def perceive_and_decide(self, observation):
        # preprocess the observation, set input to network to be difference image

        self.cur_x = self.prepro(observation['scene'])
        self.diff_x = self.get_difference_frame(self.cur_x, self.prev_x)
        # save the current from for the next iterations previous frame
        self.prev_x = self.cur_x
        # get e-greedy predicted action,probability and error
        aprob, h, action = self.get_action_egreedy(self.diff_x)

        if action == 0:  # turn left
            steer = -1
            accel = 0
        elif action == 1:  # turn left, accelerate
            steer = -1
            accel = 1
        elif action == 2:  # accelerate
            steer = 0
            accel = 1
        elif action == 3:  # turn right, accelerate
            steer = 1
            accel = 1
        elif action == 4:  # turn right
            steer = 1
            accel = 0
        else:  # decelerate
            steer = 0
            accel = -1

        return h, steer, accel, self.get_error(action, aprob)

    def update_reward(self, h, reward, error):
        self.lastReward = reward
        self.reward_sum += reward
        # print "action,reward, done = " + str(action) + "," + str(reward) + "," + str(done)
        # store values for use in backpropagation later
        self.store_values(self.diff_x, h, error, reward)

    def finish_episode(self):
        print("episode finished")
        # observation = None
        # gc.collect()

        self.episode_number += 1

        # stack together all inputs, hidden states, action gradients, and rewards for this episode
        episode_x = np.vstack(self.xs)
        episode_h = np.vstack(self.hs)
        episode_dlogp = np.vstack(self.dlogps)
        episode_rewards = np.vstack(self.drs)
        self.xs, self.hs, self.dlogps, self.drs = [], [], [], []  # reset episode array memory

        rr = self.discount_rewards(episode_rewards)
        print("discounted reward: %s" % str(rr))

        # standardize the rewards to be unit normal (helps control the gradient estimator variance)
        rr -= np.mean(rr)
        rr /= np.std(rr)

        episode_dlogp *= rr  # modulate the gradient with advantage (PG magic happens right here.)
        grad = self.get_policy_backward(episode_h, episode_dlogp, episode_x)
        for k in self.model:
            self.grad_buffer[k] += grad[k]  # accumulate grad over batch
            print("grad_buffer[k] length = " + str(self.rmsprop_cache[k].shape))

        print("W1 Taps" + str(self.model['W1'].shape))
        print("W2 Taps" + str(self.model['W2'].shape))
        # perform rmsprop parameter update every batch_size episodes

        if self.episode_number % self.batch_size == 0:
            for k, v in self.model.items():
                self.grad_buffer[k] = np.zeros_like(v)  # reset batch gradient buffer
                print("k,v = " + str(len(k)) + "," + str(len(v)))
                g = self.grad_buffer[k]  # gradient
                print("rmsprop_cache,g = " + str(self.rmsprop_cache[k].shape) + "," + str(g.shape))
                self.rmsprop_cache[k] = self.decay_rate * self.rmsprop_cache[k] + (1 - self.decay_rate) * g ** 2

                # print("model[k] length,data = " + str(len(model[k])) + "," + str(model[k]))
                # print("g length,data = " + str(len(g)) + "," + str(g))
                # print("rmsprop_cache,data = " + str(len(self.rmsprop_cache[k])) + "," + str(self.rmsprop_cache[k]))
                self.model[k] += self.learning_rate * g / (np.sqrt(self.rmsprop_cache[k]) + 1e-5)
                self.grad_buffer[k] = np.zeros_like(v)  # reset batch gradient buffer

        # boring book-keeping
        self.running_reward = self.reward_sum if self.running_reward is None else self.running_reward * 0.99 + self.reward_sum * 0.01
        print('resetting env. episode reward total was %f. running mean: %f' % (self.reward_sum, self.running_reward))

        if self.episode_number % 100 == 0:
            f = open('save.p', 'wb')
            print(len(self.model['W1']))
            print(len(self.model['W1'][0]))
            pickle.dump(self.model['W1'][:50], f)
            pickle.dump(self.model['W1'][50:100], f)
            pickle.dump(self.model['W1'][100:150], f)
            pickle.dump(self.model['W1'][150:], f)
            pickle.dump(self.model['W2'], f)
        # reward_sum = 0
        # observation = env.reset()  # reset env
        self.prev_x = np.zeros(self.dimensionality)


class TrainingVehicle(SimpleVehicle):
    def __init__(self, simulator_config, vehicle_config, agent_config, brain):
        super(TrainingVehicle, self).__init__(simulator_config, vehicle_config)
        self.simulator_config = simulator_config
        self.vehicle_config = vehicle_config
        self.config = agent_config.configuration
        self.brain = brain

        self.reset()

    def reset(self):
        self.should_restart_vehicle = False
        self.brain.reset()

        # Display Screen
        if self.config["render"]:
            self.screen = cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('image', 320, 320)
        else:
            self.screen = None

    def render(self, observation):
        scene = observation.get('frame')
        if scene is None:
            print("Dropped packets")
            return

        # last_frame = cv2.cvtColor(last_frame, cv2.COLOR_BGRA2GRAY)
        cv2.imshow('image', scene)
        cv2.waitKey(1)

    def restart_vehicle(self):
        self.restart_event.set()

    def run(self):
        try:
            # MonoDrive Init
            observation, reward, done = self.step(0, 0)

            while not done:
                if self.config["render"]:
                    self.render(observation)

                h, accel, steer, dError = self.brain.perceive_and_decide(observation)
                # print("--> %d, %f %f" % (action, accel, steer))
                observation, rewards, done = self.step({'forward': accel, 'right': steer})
                reward = 0.0
                for r in rewards:
                    reward = reward + float(r)
                    if r < 0:
                        reward = float(r)

                self.brain.update_reward(h, reward, dError)

                if done:  # an episode finished
                    self.brain.finish_episode()

                    self.restart_vehicle()

        except Exception as e:
            print(str(e))
            traceback.print_exc(file=sys.stdout)
        except KeyboardInterrupt:
            raise RuntimeError('KeyboardInterrupt')
        finally:
            print("exit!")

    def step(self, control_data):
        super(TrainingVehicle, self).step(control_data)

        control_data = super(TrainingVehicle, self).drive(self.sensors)

        observation = None
        for sensor in self.sensors:
            if sensor.type == 'Camera':
                image = sensor.get_q_image()
                observation = {
                    "scene": cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                        (self.config['image_width'], self.config['image_height']))
                }
                break

        reward, done = self.score(control_data)
        return observation, [reward], done

    def score(self, control_data):
        control_data = self.get_lane_offset_and_speed()
        print('offset: ', control_data['offset'])
        reward = -math.fabs(control_data['offset']) - math.fabs(control_data['speed_limit'] - control_data['speed'])

        done = control_data['restart'] or self.should_restart_vehicle or self.brain.reward_sum < -1000

        print(reward, done)
        return reward, done

    def get_lane_offset_and_speed(self):
        offset = 0
        speed_limit = 0
        speed = self.velocity
        if self.waypoint_msg:
            speed_limit = self.waypoint_msg.speed_limit_by_lane[self.waypoint_msg.lane_number]

        return {
            'offset': offset,
            'speed_limit': speed_limit,
            'speed': speed,
            'restart': False
        }

if __name__ == "__main__":
    simulator_config = SimulatorConfiguration('simulator.json')
    vehicle_config = VehicleConfiguration('test.json')
    agent_config = Configuration('../examples/agent_trainer_config.json')

    dim = int(agent_config.configuration["image_width"] / agent_config.configuration["down_sample"] * \
              agent_config.configuration["image_height"] / agent_config.configuration["down_sample"])
    brain = SimpleNeuralNetwork(hidden_layers=agent_config.configuration['hidden_layers'],
                                outputs=agent_config.configuration['outputs'],
                                dimensionality=dim, gamma=agent_config.configuration['gamma'],
                                down_sample=agent_config.configuration['down_sample'],
                                batch_size=agent_config.configuration['batch_size'],
                                decay_rate=agent_config.configuration['decay_rate'],
                                learning_rate=agent_config.configuration['learning_rate'],
                                resume=agent_config.configuration['resume'])

    simulator = Simulator(simulator_config)
    simulator.send_configuration()

    cycles = 0
    while cycles < agent_config.configuration['training_cycles']:
        # Create vehicle process
        vehicle_process = TrainingVehicle(simulator, vehicle_config, agent_config, brain)

        simulator.restart_event.clear()
        simulator.send_vehicle_configuration(vehicle_config)

        # Start the Vehicle process
        vehicle_process.start()

        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        # Terminates control process
        vehicle_process.stop()

        cycles = cycles + 1

