# gui/model_training.py
import threading
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import torch
import torch.nn as nn
from gym import Env, spaces
import numpy as np
import joblib

class LSTMExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=128):
        super(LSTMExtractor, self).__init__(observation_space, features_dim)
        self.lstm = nn.LSTM(input_size=observation_space.shape[1], hidden_size=features_dim, batch_first=True)
        self.linear = nn.Linear(features_dim, features_dim)

    def forward(self, observations):
        lstm_out, _ = self.lstm(observations)
        return self.linear(lstm_out[:, -1, :])

class CustomLSTMPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomLSTMPolicy, self).__init__(*args, **kwargs, features_extractor_class=LSTMExtractor, features_extractor_kwargs=dict(features_dim=128))

class EnhancedSensorEnv(Env):
    def __init__(self, data, history_length=5):
        super(EnhancedSensorEnv, self).__init__()
        self.data = data
        self.history_length = history_length
        self.current_step = 0

        # Observation and action space
        sensor_columns = ['CH1', 'CH2', 'CH3', 'hour', 'day_of_week', 'is_weekend']
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.history_length, len(sensor_columns)), dtype=np.float32)
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)

    def reset(self):
        self.current_step = self.history_length
        return self._get_observation()

    def _get_observation(self):
        return self.data.iloc[self.current_step - self.history_length:self.current_step].values

    def step(self, action):
        true_values = self.data.iloc[self.current_step][['CH1', 'CH2', 'CH3']].values
        reward = -np.mean(np.abs(action - true_values))
        self.current_step += 1
        done = self.current_step >= len(self.data)
        return self._get_observation(), reward, done, {}

class ModelTraining:
    def __init__(self, visualization):
        self.model = None
        self.visualization = visualization

    def train_new_model(self, data):
        # Setup RL Environment
        env = EnhancedSensorEnv(data)
        check_env(env)

        # Initialize the model
        self.model = PPO(CustomLSTMPolicy, env, verbose=1)

        # Training in a separate thread
        training_thread = threading.Thread(target=self._train_model, args=(env,))
        training_thread.start()

    def _train_model(self, env):
        self.model.learn(total_timesteps=10000)
        self.visualization.plot_training_results(env, self.model)

    def fine_tune_model(self, data):
        if self.model is None:
            QMessageBox.warning(self.visualization.parent, "Error", "Train a model first!")
            return
        env = EnhancedSensorEnv(data)
        training_thread = threading.Thread(target=self._train_model, args=(env,))
        training_thread.start()

    def test_model(self, real_time_data):
        obs = real_time_data.iloc[:self.env.history_length].values
        predictions = []

        for i in range(self.env.history_length, len(real_time_data)):
            action, _ = self.model.predict(obs)
            predictions.append(action)
            obs = real_time_data.iloc[i-self.env.history_length:i].values

        prediction_df = pd.DataFrame(predictions, columns=["Predicted CO2", "Predicted Temp", "Predicted Humidity"])
        return pd.concat([real_time_data.reset_index(drop=True), prediction_df], axis=1)

    def save_model(self, file_path):
        if self.model is not None:
            joblib.dump(self.model, file_path)
