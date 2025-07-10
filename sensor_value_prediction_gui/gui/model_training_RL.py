# gui/model_training_RL.py
from PyQt5.QtCore import pyqtSignal
import joblib
import pandas as pd
import threading
from gymnasium.utils.env_checker import check_env
from stable_baselines3 import PPO
from gui.utils.environment import EnhancedSensorEnv
from gui.models.ppo_model import CustomLSTMPolicy

from PyQt5.QtCore import pyqtSignal, QObject  # Import QObject

class ModelTraining(QObject):  # Inherit from QObject
    progress_signal = pyqtSignal(int)  # Define the signal

    def __init__(self, visualization):
        super().__init__()  # Initialize QObject
        self.model = None
        self.visualization = visualization

    def train_new_model(self, data, progress_callback):
        # Setup RL Environment
        env = EnhancedSensorEnv(data)
        check_env(env)  # Check if the environment is valid

        # Initialize the model with PPO and custom LSTM policy
        self.model = PPO(CustomLSTMPolicy, env, verbose=1)

        # Start the training process in a separate thread
        training_thread = threading.Thread(target=self._train_model, args=(env, progress_callback))
        training_thread.start()

    def _train_model(self, env, progress_callback):
        # Train the PPO model
        total_timesteps = 10000
        progress_interval = total_timesteps // 100  # Update progress bar every 1% of training
        for timestep in range(0, total_timesteps, progress_interval):
            self.model.learn(total_timesteps=progress_interval)
            self.progress_signal.emit(int((timestep / total_timesteps) * 100))  # Emit progress
        self.visualization.plot_training_results(env, self.model)
        self.progress_signal.emit(100)  # Ensure progress reaches 100%

    def fine_tune_model(self, data):
        if self.model is None:
            QMessageBox.warning(self.visualization.parent, "Error", "Train a model first!")
            return
        env = EnhancedSensorEnv(data)
        fine_tune_thread = threading.Thread(target=self._fine_tune_model, args=(env,))
        fine_tune_thread.start()

    def _fine_tune_model(self, env):
        self.model.learn(total_timesteps=5000)  # Fine-tune with additional timesteps
        self.visualization.plot_training_results(env, self.model)

    def test_model(self, real_time_data):
        # Prepare data for testing
        obs = real_time_data.iloc[:self.model.env.history_length].values  # Use the initial history length for testing
        predictions = []

        # Generate predictions using the trained model
        for i in range(self.model.env.history_length, len(real_time_data)):
            action, _ = self.model.predict(obs)
            predictions.append(action)
            obs = real_time_data.iloc[i - self.model.env.history_length:i].values  # Move window forward

        # Return predictions along with the real data
        prediction_df = pd.DataFrame(predictions, columns=["Predicted CO2", "Predicted Temp", "Predicted Humidity"])
        return pd.concat([real_time_data.reset_index(drop=True), prediction_df], axis=1)

    def save_model(self, file_path):
        if self.model is not None:
            joblib.dump(self.model, file_path)
        else:
            raise ValueError("No model to save!")

    def save_predictions_to_csv(self, predictions, file_path):
        predictions.to_csv(file_path, index=False)
