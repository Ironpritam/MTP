# gui/model_training.py

from PyQt5.QtCore import pyqtSignal, QObject , QThread
from PyQt5.QtWidgets import QMessageBox
# import threading
import torch
from sklearn.metrics import mean_absolute_percentage_error, mean_squared_error
import pandas as pd
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, TensorDataset
from gui.models.gru_model import GRURegressionModel
from gui.data_loader import DataLoader as CustomDataLoader  # Avoid naming conflict
from gui.data_loader import save_splits, update_time_features, save_original



class TrainingThread(QThread):
    progress_signal = pyqtSignal(int)  # Signal to update progress in GUI
    task_finished = pyqtSignal(object)
    def __init__(self, model_trainer, train_loader, test_loader, progress_callback=None):
        super().__init__()
        self.model_trainer = model_trainer
        self.train_loader = train_loader
        self.test_loader = test_loader
        # self.epochs = epochs
        self.progress_callback = progress_callback

    def run(self):
        # Run the training process without emitting progress at the end since it is handled in _train_model
        self.model_trainer._train_model(self.train_loader, self.test_loader, self.progress_signal.emit)
        self.task_finished.emit(self.model_trainer.model)



class ModelTraining(QObject):
    progress_signal = pyqtSignal(int)  # Signal to update GUI progress bar
    task_finished = pyqtSignal(object)

    def __init__(self, visualization):
        super().__init__()  # Initialize QObject
        self.model = None
        self.visualization = visualization
        self.data_loader = CustomDataLoader()  # Initialize the custom DataLoader
        self.pred_sensors = len(self.data_loader.sensor_columns)

    def train_new_model(self, data, input_length=30, output_length=4, batch_size=32,epochs=30, learning_rate=0.001, split=0.2, progress_callback=None):
        """
        Train a new GRU model using the preloaded data.
        """
        # Load and preprocess data
        self.data_loader.data = data
        self.epochs = epochs
        self.learning_rate = learning_rate
        self.split = split
        save_original(data,self.split)
        self.data_loader._preprocess_data()
        self.data_loader.data = self.data_loader.data.drop(columns=['Timestamp'])

        # Prepare time-series datasets
        X, y = self.data_loader.prepare_datasets(input_length, output_length,self.pred_sensors)

        # Split into training and testing datasets
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, shuffle=False)

        save_splits(X_train, X_test, y_train, y_test)

        # Create PyTorch DataLoaders
        train_loader = DataLoader(TensorDataset(torch.tensor(X_train, dtype=torch.float32),
                                                torch.tensor(y_train, dtype=torch.float32)),
                                  batch_size=batch_size, shuffle=True)
        test_loader = DataLoader(TensorDataset(torch.tensor(X_test, dtype=torch.float32),
                                               torch.tensor(y_test, dtype=torch.float32)),
                                 batch_size=batch_size, shuffle=False)

        # Model parameters
        # print(y[0])
        # print(y.shape)
        # print(X.shape)
        input_size = X.shape[2]  # Number of features
        hidden_size = 128  # GRU hidden state size
        num_layers = 2  # Number of GRU layers
        output_size = self.pred_sensors  #*output_length  # Predict `output_length` future points

        # Initialize the model
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = GRURegressionModel(input_size, hidden_size, num_layers, output_size).to(
            device
        )
        device_message = f"Model will run on: {str(device)}"
        print(device_message)
        # Trying to pop device info on messagebox
        # QMessageBox.information(self.visualization, "Model will run on",device_message)

        # Start training in a separate thread (using Thread module)
        # training_thread = threading.Thread(target=self._train_model, args=(train_loader, test_loader, epochs, progress_callback))
        # training_thread.start()
        # training_thread.join()
        # self.task_finished.emit(self.model)
        self.training_thread = TrainingThread(self, train_loader, test_loader)
        self.training_thread.progress_signal.connect(self.progress_signal.emit)
        self.training_thread.task_finished.connect(self.task_finished.emit)
        self.training_thread.start()



    def _train_model(self, train_loader, test_loader, progress_callback):
        """
        Internal method to train the GRU model.
        """
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(device)
        criterion = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)

        # num_epochs = self.epochs
        total_steps = len(train_loader) * self.epochs
        step_counter = 0

        for epoch in range(self.epochs):
            self.model.train()
            train_loss = 0.0
            train_accuracy = 0.0

            for inputs, targets in train_loader:
                inputs, targets = inputs.to(device), targets.to(device)

                # Forward pass
                predictions = self.model(inputs)
                loss = criterion(predictions, targets)

                # Backward pass
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                train_loss += loss.item()
                # Calculate accuracy (e.g., MAPE)
                train_accuracy += mean_absolute_percentage_error(targets.cpu().numpy(), predictions.cpu().detach().numpy())
                step_counter += 1

                if progress_callback:
                    self.progress_signal.emit(int((step_counter / total_steps) * 100))  # Emit progress

            # Evaluate after each epoch
            val_loss, val_accuracy = self._evaluate_model(test_loader, criterion)
            print(f"Epoch [{epoch + 1}/{self.epochs}] - Train Loss: {train_loss / len(train_loader):.4f}, "
                  f"Train MAPE: {train_accuracy / len(train_loader):.4f}, "
                  f"Validation Loss: {val_loss:.4f}, Validation MAPE: {val_accuracy:.4f}")


        # self.visualization.plot_training_results(test_loader, self.model)
        if progress_callback:
             self.progress_signal.emit(100)  # Ensure progress reaches 100%
        return self.model

    def _evaluate_model(self, loader, criterion):
        """
        Evaluate the model on the provided DataLoader.
        """
        self.model.eval()
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        val_loss = 0.0
        val_accuracy = 0.0

        with torch.no_grad():
            for inputs, targets in loader:
                inputs, targets = inputs.to(device), targets.to(device)
                predictions = self.model(inputs)
                loss = criterion(predictions, targets)
                val_loss += loss.item()
                val_accuracy += mean_absolute_percentage_error(targets.cpu().numpy(), predictions.cpu().detach().numpy())

        return val_loss / len(loader), val_accuracy / len(loader)

    def test_model(self, input_length=30, prediction_steps=60, test_data=None):
        """
        Test the trained GRU model using sliding window iterative predictions.

        :param input_length: Number of past time steps used for each prediction.
        :param prediction_steps: Number of future time steps to predict iteratively.
        :param test_data: Optional. A DataFrame containing test data. If not provided, uses the loaded CSV data.
        :return: A tuple of (actual values DataFrame, predicted values DataFrame).
        """
        if self.model is None:
            raise ValueError("Model not trained yet!")

        # Check if test data is provided, else use loaded data
        if test_data is None:
            if self.data_loader.data is None:
                raise ValueError("No test data provided, and no data loaded. Please load data first.")
            test_data = self.data_loader.data

        # Ensure test_data is a Pandas DataFrame
        if not isinstance(test_data, pd.DataFrame):
            raise TypeError("Test data must be a Pandas DataFrame.")

        # Prepare sliding window for testing
        actual_values = []
        predicted_values = []

        # Initialize the sliding window
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(device)
        self.model.eval()

        num_samples = len(test_data) - input_length - prediction_steps + 1
        for i in range(num_samples):
            # Extract the input sequence and actual values for the current window
            input_seq = test_data.iloc[i : i + input_length].values
            actual_seq = test_data.iloc[i + input_length : i + input_length + prediction_steps].values

            # Perform iterative prediction
            predicted_seq = self.iterative_prediction(
                input_data=input_seq.tolist(),
                prediction_steps=prediction_steps,
                input_length=input_length,
                output_length=self.pred_sensors,
                device=device,
            )

            # Store results
            actual_values.append(actual_seq)
            predicted_values.append(predicted_seq)

        # Convert results to DataFrames
        actual_df = pd.DataFrame(
            data=np.vstack(actual_values),
            columns=[f"Actual_{col}" for col in self.data_loader.sensor_columns]
        )
        predicted_df = pd.DataFrame(
            data=np.vstack(predicted_values),
            columns=[f"Predicted_{col}" for col in self.data_loader.sensor_columns]
        )

        return actual_df, predicted_df



    def iterative_prediction(self, input_data, prediction_steps, input_length, output_length, device):
        """
        Perform iterative predictions using a sliding window approach.

        :param input_data: List of input features (already normalized).
        :param prediction_steps: Number of steps to predict iteratively.
        :param input_length: Length of the input sequence.
        :param output_length: Number of sensor outputs to predict per step.
        :param device: PyTorch device (e.g., "cpu" or "cuda").
        :return: List of predicted values.
        """
        self.model.eval()

        # Convert input data to tensor and move to device
        input_seq = torch.tensor(input_data[-input_length:], dtype=torch.float32).unsqueeze(0).to(device)
        predicted_values = []

        for _ in range(prediction_steps):
            with torch.no_grad():
                # Predict next step
                predicted_step = self.model(input_seq)
                predicted_step = predicted_step.squeeze(0).cpu().numpy()

                # Append predicted sensor values
                predicted_values.append(predicted_step[:output_length])

                # Create the next input sequence
                next_input = predicted_step

                # Extract additional time-based features
                last_known_time_features = input_seq.cpu().numpy()[0, -1, output_length:]
                updated_time_features = update_time_features(last_known_time_features,1)

                # Combine predicted sensor values with updated time features
                next_input_combined = np.concatenate([next_input, updated_time_features])

                # Update input sequence for the next iteration
                input_seq = torch.tensor(
                    np.concatenate([input_seq.cpu().numpy()[0, 1:], [next_input_combined]]),
                    dtype=torch.float32
                ).unsqueeze(0).to(device)

        return np.array(predicted_values)


    def test_model_iterative_real_time(self, input_length, prediction_steps):
        """
        Test the trained GRU model using iterative predictions.
        """
        if self.model is None:
            raise ValueError("Model not trained yet!")

        if self.data_loader.data is None:
            raise ValueError("No data loaded. Please load data before testing.")

        # Prepare the initial sequence for iterative prediction
        initial_sequence = self.data_loader.data[-input_length:]  # Take the most recent `input_length` data points
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Perform iterative prediction
        predictions = self.iterative_prediction(initial_sequence,
                                                prediction_steps=prediction_steps,
                                                input_length=input_length,
                                                device=device)

        # Create a DataFrame for predictions
        prediction_df = pd.DataFrame(predictions,
                                      columns=[f"Predicted_{col}" for col in self.data_loader.sensor_columns])

        return prediction_df


    # avoiding torch.save(model.state_dict(),path) method as it will need rewriting the model code and pass input_size ... parametes which are dynamic

    def save_model(self, model, file_path):
        """
        Save the trained GRU model to a file.
        """
        if model is None:
            raise ValueError("No model to save!")
        torch.save(model, file_path)
        comment = f"Model saved to: {file_path}"
        print(comment)
        QMessageBox.information(self.visualization, "Info.", comment)

    def load_model(self, model_path, input_size, hidden_size, num_layers, output_size):
        """
        Load a pre-trained model from a file.
        """
        # self.model = self.create_rnn_model(input_size, hidden_size, num_layers, output_size)
        # self.model.load_state_dict(torch.load(model_path))
        self.model = torch.load(modal_path)
        self.model.eval()


    def save_predictions_to_csv(self, predictions, file_path):
        """
        Save predictions to a CSV file.
        """
        predictions.to_csv(file_path, index=False)
        print(f"Predictions saved to {file_path}")
