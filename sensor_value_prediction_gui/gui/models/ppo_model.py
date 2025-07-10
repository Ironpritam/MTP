# models/ppo_model.py
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import torch.nn as nn

# LSTM Feature Extractor for the custom policy
class LSTMExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=128):
        super(LSTMExtractor, self).__init__(observation_space, features_dim)
        self.lstm = nn.LSTM(input_size=observation_space.shape[1], hidden_size=features_dim, batch_first=True)
        self.linear = nn.Linear(features_dim,3) # features_dim) removing feature dimension since need to predict 3 values not whole feature space

    def forward(self, observations):
        lstm_out, _ = self.lstm(observations)
        return self.linear(lstm_out[:, -1, :])  # Take only the last time step output

# Custom Policy using the LSTMExtractor
class CustomLSTMPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomLSTMPolicy, self).__init__(*args, **kwargs, features_extractor_class=LSTMExtractor, features_extractor_kwargs=dict(features_dim=128))

def create_ppo_model(env):
    """
    Creates a PPO model with a custom LSTM-based policy.

    :param env: The environment to train the model on
    :return: The PPO model
    """
    model = PPO(CustomLSTMPolicy, env, verbose=1)
    return model
