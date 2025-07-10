# utils/environment.py
from gymnasium import Env, spaces
import numpy as np
from gymnasium.utils.seeding import np_random

class EnhancedSensorEnv(Env):
    def __init__(self, data, history_length=5):
        super(EnhancedSensorEnv, self).__init__()
        self.data = data
        self.history_length = history_length
        self.current_step = self.history_length  # Start from the history_length
        self.total_steps = len(self.data)

        # Observation space shape: (history_length, num_features)
        sensor_columns = ['CH1','CH2','CH3','hour', 'day_of_week', 'is_weekend']

                         #  ['Temperature', 'Humidity', 'CO2']
                         # 'hour', 'day_of_week', 'is_weekend']   for detecting day which i will look later

        self.observation_space = spaces.Box(low=0, high=1, shape=(self.history_length, len(sensor_columns)), dtype=np.float32)

        # following codeline is for general data not normalize data
        # self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.history_length, len(sensor_columns)), dtype=np.float32)

        # Define action space (Example: predicting future sensor values)
        self.action_space = spaces.Box(low=-1, high=1, shape=(len(sensor_columns),), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        # Call the parent class to handle the seeding
        super().reset(seed=seed)
        self.np_random, _ = self.seed(seed)
        self.current_step = self.history_length  # Reset to the history length position
        # Return the observation and an empty info dictionary
        observation = self._get_observation()
        observation = observation.astype(np.float32)
        info = {}  # Info can be empty or contain additional information
        return observation, info

    def seed(self, seed=None):
        self.np_random, seed = np_random(seed)
        return self.np_random, seed

    def _get_observation(self):
        # Get the previous 'history_length' steps of data
        return self.data.iloc[self.current_step - self.history_length:self.current_step].values

    def step(self, action):
        # Use the next real values to calculate the reward
        true_values = self.data.iloc[self.current_step][['CH1','CH2','CH3']].values
        reward = -np.mean(np.abs(action - true_values))  # Minimize prediction error
        self.current_step += 1
        done = self.current_step >= self.total_steps
        return self._get_observation(), reward, done, {}  # Info is an empty dict or can contain additional details
