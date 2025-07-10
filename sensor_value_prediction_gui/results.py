import torch
from sklearn.metrics import (mean_absolute_percentage_error, mean_absolute_error, mean_squared_error, r2_score)
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, TensorDataset
from gui.models.gru_model import GRURegressionModel
from gui.data_loader import DataLoader as CustomDataLoader
from gui.data_loader import update_time_features


pd.set_option('future.no_silent_downcasting', True)
sensors = ['Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)'  ]
weights_for_SmoothL1Loss = [1,1,1,1] #[3,1.1,0.8,2.5]






def plot_pred_vs_true(y_test, y_pred):
    results_dir = os.path.join(os.getcwd(), "Results")
    for i in range(4):
        plt.plot(range(0,60), y_test[:60, i], label="True {}".format(sensors[i]))
        plt.plot(range(0,60),y_pred[:60, i], label="Predicted {}".format(sensors[i]), linestyle="--")
        plt.title(f"{sensors[i]}: True vs Predicted")
        plt.xlabel("Time (Minutes)")
        plt.ylabel("Sensor Value")
        plt.legend()
        plt.savefig(os.path.join(results_dir, f"Pred_vs_true-{sensors[i]}.png"))
        # plt.show()
        plt.close()


def plot_residuals(y_test,y_pred):
    results_dir = os.path.join(os.getcwd(), "Results")
    for i in range(4):
        residuals = y_test[:, i] - y_pred[:, i]
        plt.scatter(range(len(residuals)), residuals, alpha=0.6)
        plt.axhline(0, color='red', linestyle='--')
        plt.title(f"Residuals for {sensors[i]}")
        plt.xlabel("Time (Minutes)")
        plt.ylabel("Residual")
        plt.savefig(os.path.join(results_dir,f"Residuals-{sensors[i]}.png"))
        # plt.show()
        plt.close()


def plot_training_results_loss(model_training):
    results_dir = os.path.join(os.getcwd(), "Results")
    plt.plot(model_training.train_losses, label='Training Loss')
    plt.plot(model_training.val_losses, label='Validation Loss')
    plt.legend()
    plt.title("Learning Curve")
    plt.xlabel("Epochs")
    plt.ylabel("Loss")
    plt.savefig(os.path.join(results_dir,"Training_Results_Loss.png"))
    # plt.show()
    plt.close()


def plot_training_results_accuracy(model_training):
    results_dir = os.path.join(os.getcwd(), "Results")
    plt.plot(model_training.train_accuracies, label='Training Loss')
    plt.plot(model_training.val_accuracies, label='Validation Loss')
    plt.legend()
    plt.title("Mean Absolute Percentage Error")
    plt.xlabel("Epochs")
    plt.ylabel("Accuracy")
    plt.savefig(os.path.join(results_dir,"Training_Results_Accuracy.png"))
    # plt.show()
    plt.close()


def adjusted_r2_score(y_true, y_pred, num_predictors):
    """
    Calculate Adjusted R-squared.
    Parameters:
    y_true: ndarray
        True values of the target variable.
    y_pred: ndarray
        Predicted values of the target variable.
    num_predictors: int
        Number of predictors used in the model.
    Returns:
    float
        Adjusted R-squared value.
    """
    # Calculate R-squared
    r2 = r2_score(y_true, y_pred)
    # Number of observations
    n = len(y_true)
    # Calculate Adjusted R-squared
    adjusted_r2 = 1 - ((1 - r2) * (n - 1) / (n - num_predictors - 1))
    return adjusted_r2


def adjusted_r2_multioutput(y_true, y_pred, num_predictors):
    """
    Calculate Adjusted R-squared for multi-output regression.
    """
    n = y_true.shape[0]
    r2_scores = r2_score(y_true, y_pred, multioutput='raw_values')
    adjusted_r2_scores = [
        1 - ((1 - r2) * (n - 1) / (n - num_predictors - 1)) for r2 in r2_scores
    ]
    return np.mean(adjusted_r2_scores)




# class WeightedMSELoss(torch.nn.Module):
#     def __init__(self, weights):
#         super(WeightedMSELoss, self).__init__()
#         self.weights = torch.tensor(weights, dtype=torch.float32)
#
#     def forward(self, predictions, targets):
#         loss = (self.weights * (predictions - targets) ** 2).mean()
#         return loss


class CombinedEarlyStopping:
    def __init__(self, patience=5, delta=0, save_path="best_model.pth", warmup_epochs=10):
        """
        Early stopping based on a combined score of validation loss and MAPE, with a warm-up period.
        :param patience: Number of epochs to wait before stopping.
        :param delta: Minimum change to qualify as an improvement.
        :param save_path: Path to save the best model.
        :param warmup_epochs: Number of epochs to collect stats before starting early stopping.
        """
        self.patience = patience
        self.delta = delta
        self.save_path = save_path
        self.best_score = float('inf')
        self.counter = 0
        self.early_stop = False
        self.max_val_loss = None  # Will be initialized after warm-up
        self.max_val_mape = None  # Will be initialized after warm-up
        self.warmup_epochs = warmup_epochs
        self.current_epoch = 0

    def __call__(self, val_loss, val_mape, model):
        self.current_epoch += 1

        # During warm-up, collect maximum metrics for scaling
        if self.current_epoch <= self.warmup_epochs:
            if self.max_val_loss is None or val_loss > self.max_val_loss:
                self.max_val_loss = val_loss
            if self.max_val_mape is None or val_mape > self.max_val_mape:
                self.max_val_mape = val_mape
            print(f"Warm-up Epoch [{self.current_epoch}/{self.warmup_epochs}] - Collecting max stats")
            return  # Skip early stopping during warm-up

        # Normalize metrics using max values collected during warm-up
        normalized_loss = val_loss / (self.max_val_loss + 1e-8)
        normalized_mape = val_mape / (self.max_val_mape + 1e-8)

        # Compute combined score
        combined_score = normalized_loss + normalized_mape

        # Check for improvement
        if combined_score < self.best_score - self.delta:
            self.best_score = combined_score
            self.counter = 0
            torch.save(model, self.save_path)
        else:
            self.counter += 1
            if self.counter >= self.patience:
                self.early_stop = True


class EarlyStopping:
    def __init__(self, patience=5, delta=0, save_path="best_model11.pth"):
        """
        Early stopping to terminate training when validation loss stops improving.

        :param patience: Number of epochs to wait before stopping.
        :param delta: Minimum change in the validation loss to qualify as improvement.
        :param save_path: Path to save the best model.
        """
        self.patience = patience
        self.delta = delta
        self.save_path = save_path
        self.best_loss = float('inf')
        self.counter = 0
        self.early_stop = False

    def __call__(self, val_loss, model):
        if val_loss < self.best_loss - self.delta:
            self.best_loss = val_loss
            self.counter = 0
            torch.save(model, self.save_path)
        elif val_loss >= self.best_loss :
            self.counter += 1
            if self.counter >= self.patience:
                self.early_stop = True



class WeightedSmoothL1Loss(torch.nn.Module):
    def __init__(self, weights):
        """
        Custom SmoothL1Loss with weights for each target feature.

        :param weights: List or tensor of weights corresponding to the target features.
        """
        super(WeightedSmoothL1Loss, self).__init__()
        self.weights = torch.tensor(weights, dtype=torch.float32)
        self.smooth_l1_loss = torch.nn.SmoothL1Loss(reduction='none')

    def forward(self, predictions, targets):
        """
        Compute weighted SmoothL1Loss.

        :param predictions: Predicted values from the model (batch_size, num_features).
        :param targets: Ground truth values (batch_size, num_features).
        :return: Weighted loss scalar.
        """
        # Compute the element-wise SmoothL1Loss
        losses = self.smooth_l1_loss(predictions, targets)  # Shape: (batch_size, num_features)

        # Apply weights to each feature's loss
        weighted_losses = losses * self.weights.to(losses.device)  # Shape: (batch_size, num_features)

        # Average the weighted losses across features and batch
        return weighted_losses.mean()


class ModelTraining():


    def __init__(self):
        super().__init__()  # Initialize QObject
        self.model = None
        self.data_loader = CustomDataLoader()  # Initialize the custom DataLoader
        self.pred_sensors = 4
        self.prediction_sensors = ['Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)']

    def train_new_model(self, data, input_length=40, output_length=4, batch_size=32,epochs=30, learning_rate=0.001, split=0.2, progress_callback=None): # /// replaced data with data_loader
        """
        Train a new GRU model using the preloaded data.
        """
        # Load and preprocess data
        self.data_loader = data
        self.epochs = epochs
        self.learning_rate = learning_rate
        self.split = split
        save_original(self.data_loader.data,self.split)
        self.data_loader._preprocess_data()
        self.data_loader.data = self.data_loader.data.drop(columns=['Timestamp'])

        # Prepare time-series datasets
        X, y = self.data_loader.prepare_datasets(input_length, output_length,self.pred_sensors)

        # Split into training and testing datasets
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, shuffle=False)

        # save_splits(X_train, X_test, y_train, y_test)

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
        hidden_size = 64  # GRU hidden state size
        num_layers = 2  # Number of GRU layers
        output_size = self.pred_sensors  #*output_length  # Predict `output_length` future points
        dropout = 0.2

        # Initialize the model
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = GRURegressionModel(input_size, hidden_size, num_layers, output_size, dropout=dropout).to(
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
        self._train_model(train_loader, test_loader)



    def _train_model(self, train_loader, test_loader, patience=12, progress_callback=None):
        """
        Internal method to train the GRU model.
        """
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(device)

        # /// weighted_smooth_l1_loss custome start
        weighted_smooth_l1_loss = WeightedSmoothL1Loss(weights_for_SmoothL1Loss)
        criterion = weighted_smooth_l1_loss

        # criterion = torch.nn.SmoothL1Loss()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate,  weight_decay=1e-5)

            # Early stopping and scheduler setup
        early_stopping = CombinedEarlyStopping(patience=patience, save_path="best_model11.pth")
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', patience=6, factor=0.1)

        # num_epochs = self.epochs
        total_steps = len(train_loader) * self.epochs
        step_counter = 0
        self.val_losses = []
        self.val_accuracies = []
        self.train_losses = []
        self.train_accuracies = []
        scalers = self.data_loader.scalers  # Dictionary of scalers
        columns_to_transform = list(scalers.keys())[:4]  # Get the first 4 scaler keys

        for epoch in range(self.epochs):
            self.model.train()
            train_loss = 0.0
            train_accuracy = 0.0

            for inputs, targets in train_loader:
                inputs, targets = inputs.to(device), targets.to(device)

                # Forward pass
                predictions, _ = self.model(inputs)
                loss = criterion(predictions, targets)

                # Backward pass
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                train_loss += loss.item()
                # Calculate accuracy (e.g., MAPE)
                # Convert PyTorch tensors to NumPy arrays for compatibility with scalers
                targets_np = targets.cpu().numpy()
                predictions_np = predictions.cpu().detach().numpy()

                # Apply inverse transform column-wise
                for i, col in enumerate(columns_to_transform):
                    targets_np[:, i] = scalers[col].inverse_transform(targets_np[:, i].reshape(-1, 1)).flatten()
                    predictions_np[:, i] = scalers[col].inverse_transform(predictions_np[:, i].reshape(-1, 1)).flatten()

                # Calculate Mean Absolute Percentage Error (MAPE)
                train_accuracy += mean_absolute_percentage_error(targets_np, predictions_np)

                # train_accuracy += mean_absolute_percentage_error(self.data_loader.yscaler.inverse_transform(targets.cpu().numpy()),
                #                                         self.data_loader.yscaler.inverse_transform(predictions.cpu().detach().numpy()))
                step_counter += 1
                #
                # if progress_callback:
                #     self.progress_signal.emit(int((step_counter / total_steps) * 100))  # Emit progress

            # Evaluate after each epoch
            val_loss, val_accuracy = self._evaluate_model(test_loader, criterion)
            train_loss = train_loss / len(train_loader)
            train_accuracy = train_accuracy / len(train_loader)

            self.val_losses.append(val_loss)
            self.val_accuracies.append(val_accuracy)
            self.train_losses.append(train_loss)
            self.train_accuracies.append(train_accuracy)

            print(f"Epoch [{epoch + 1}/{self.epochs}] - Train Loss: {train_loss}, "
                  f"Train MAPE: {train_accuracy}, "
                  f"Validation Loss: {val_loss}, Validation MAPE: {val_accuracy}")


            # Step the scheduler
            scheduler.step(val_loss)
            # Early stopping
            early_stopping(val_loss, val_accuracy,self.model)  # /// modified for CombinedEarlyStopping
            if early_stopping.early_stop:
                print("Early stopping triggered.")
                break


        # self.visualization.plot_training_results(test_loader, self.model)
        # if progress_callback:
        #      self.progress_signal.emit(100)  # Ensure progress reaches 100%
        return self.model

    def _evaluate_model(self, loader, criterion):
        """
        Evaluate the model on the provided DataLoader.
        """
        scalers = self.data_loader.scalers  # Dictionary of scalers
        columns_to_transform = list(scalers.keys())[:4]  # Get the first 4 scaler keys
        self.model.eval()
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        val_loss = 0.0
        val_accuracy = 0.0

        with torch.no_grad():
            for inputs, targets in loader:
                inputs, targets = inputs.to(device), targets.to(device)
                predictions, _ = self.model(inputs)
                loss = criterion(predictions, targets)
                val_loss += loss.item()

                targets_np = targets.cpu().numpy()
                predictions_np = predictions.cpu().detach().numpy()
                for i, col in enumerate(columns_to_transform):
                    targets_np[:, i] = scalers[col].inverse_transform(targets_np[:, i].reshape(-1, 1)).flatten()
                    predictions_np[:, i] = scalers[col].inverse_transform(predictions_np[:, i].reshape(-1, 1)).flatten()

                val_accuracy += mean_absolute_percentage_error(targets_np, predictions_np)

        return val_loss / len(loader), val_accuracy / len(loader)

    def test_model(self, test_data=None, input_length=40, prediction_steps=30):
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


        # Feature Engineering code
        self.data_loader.data = test_data
        self.data_loader._preprocess_data()
        self.data_loader.data = self.data_loader.data.drop(columns=['Timestamp'])
        # print(self.data_loader.data.columns)
        # Normalize only the original sensor columns
        # sensor_columns = ['Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)']
        # test_data[sensor_columns] = self.data_loader.xscaler.transform(test_data[sensor_columns])

        # /// FE END



        # Initialize the sliding window
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(device)
        self.model.eval()

        #  /// for testing purpose, remove for getting real results
        self.data_loader.data = self.data_loader.data.iloc[0:420]

        num_samples = len(self.data_loader.data) - input_length - prediction_steps + 1

        # test_data = test_data.drop(columns=['Timestamp'])
        input_data = self.data_loader.data.values  # Convert DataFrame to numpy array
        # Iterate over samples
        with torch.no_grad():
            for i in range(num_samples):
                # Extract input sequence and actual future values
                input_seq = torch.tensor(input_data[i : i + input_length], dtype=torch.float32).unsqueeze(0).to(device)
                actual_seq = input_data[i + input_length : i + input_length + prediction_steps, :]

                # Perform iterative prediction
                predicted_seq = self.iterative_prediction(
                    input_data=input_seq,
                    prediction_steps=prediction_steps,
                    input_length=input_length,
                    pred_sensors_count=4,
                    device=device,
                )

                # Store predictions and actuals
                actual_values.append(np.hstack([actual_seq[:, :4], actual_seq[:, -4:]]))
                predicted_values.append(predicted_seq)

                if i % 10 == 0:  # Log progress for every 10 iterations
                    print(f"Completed prediction slot {i + 1}/{num_samples}")

        # Convert results to numpy arrays
        actual_array = np.vstack(actual_values)
        predicted_array = np.vstack(predicted_values)

        # Inverse transform actual and predicted values
        # actual_original = self.data_loader.scaler.inverse_transform(actual_array[:, :self.pred_sensors])
        # predicted_original = self.data_loader.scaler.inverse_transform(predicted_array[:, :self.pred_sensors])
        #
        # actual_original = np.hstack([actual_original,actual_array[:,-4:]])
        # predicted_original = np.hstack([predicted_original,predicted_array[:,-4:]])

        # Convert results to DataFrames
        time_columns = ['hour', 'minute', 'day_of_week', 'is_weekend']
        actual_df = pd.DataFrame(
            data=actual_array,
            columns=[f"Actual_{col}" for col in self.data_loader.sensor_columns[:self.pred_sensors]]+time_columns
        )
        predicted_df = pd.DataFrame(
            data=predicted_array,
            columns=[f"Predicted_{col}" for col in self.data_loader.sensor_columns[:self.pred_sensors]]+time_columns
        )

        for column in self.prediction_sensors:
            scaler = self.data_loader.scalers[column]
            column1 = "Actual_"+column
            actual_df[column1] = scaler.inverse_transform(actual_df[[column1]])
            column1 = "Predicted_"+column
            predicted_df[column1] = scaler.inverse_transform(predicted_df[[column1]])

        base_dir = os.path.join(os.getcwd(), "Predictions")
        os.makedirs(base_dir,exist_ok=True)

        actual_df.to_csv(os.path.join(base_dir, "actual.csv"), index=False)
        predicted_df.to_csv(os.path.join(base_dir, "predicted.csv"), index=False)

        actual_df = actual_df.drop(columns=[col for col in time_columns], errors='ignore')
        predicted_df = predicted_df.drop(columns=[col for col in time_columns], errors='ignore')

        return actual_df, predicted_df


    def _recalculate_engineered_features(self, last_input_row, predicted, pred_sensors_count=4):
        """
        Recalculate engineered features based on the last input row and the predicted values.

        :param last_input_row: The last row of the current input sequence (numpy array).
        :param predicted: The predicted sensor values (numpy array).
        :param pred_sensors_count: Number of original sensor features.
        :return: Array of recalculated engineered features.
        """
        engineered_features = []

        # Extract rolling means, rolling std, and rates of change
        for i in range(pred_sensors_count):
            # Rolling mean: Average of last value and predicted value
            rolling_mean = (last_input_row[i] + predicted[i]) / 2
            engineered_features.append(rolling_mean)

            # Rolling std: Approximation using only the last two values
            rolling_std = np.std([last_input_row[i], predicted[i]])
            engineered_features.append(rolling_std)

            # Rate of change: Difference between predicted and last value
            rate_of_change = predicted[i] - last_input_row[i]
            engineered_features.append(rate_of_change)

            # Detrended value: Predicted - rolling mean
            detrended_value = predicted[i] - rolling_mean
            engineered_features.append(detrended_value)

        return np.array(engineered_features, dtype=np.float32)



    def iterative_prediction(self, input_data, prediction_steps, input_length, pred_sensors_count=4, device='cpu'):
        """
        Perform iterative predictions for the given number of steps.

        :param input_data: Initial input sequence (2D array: [sequence_length, num_features]).
        :param prediction_steps: Number of future steps to predict iteratively.
        :param input_length: Length of the input sequence for each prediction.
        :param pred_sensors_count: Number of original sensor features predicted by the model.
        :param device: Torch device to run predictions on.
        :return: Array of predicted values (2D: [prediction_steps, pred_sensors_count]).
        """
        # Initialize predictions array
        input_seq = input_data.clone()  # Avoid modifying the original input data
        predicted_values = []

        # Iterate for the required number of prediction steps
        self.model.eval()
        with torch.no_grad():
            for step in range(prediction_steps):
                # Forward pass for the current input sequence
                predicted, attention_weights = self.model(input_seq)

                # Prepare the next input by shifting the sequence
                next_input = predicted.cpu().numpy().squeeze()

                # Extract additional time-based features
                # print(next_input.shape)
                last_known_time_features = input_seq[:,1:,:].cpu().numpy()[0, -1, -4:]
                updated_time_features = update_time_features(last_known_time_features,1)

                # Store the predicted values
                predicted_values.append(np.concatenate([next_input, updated_time_features]))


                # Recalculate features dynamically for the next step
                next_input = np.concatenate([
                    next_input,
                    self._recalculate_engineered_features(
                        input_seq[0, -1].cpu().numpy(),  # Last input timestep
                        next_input,                     # Predicted values
                        pred_sensors_count
                    )
                ]).astype(np.float32)


                # Combine predicted sensor values with updated time features
                next_input = np.concatenate([next_input, updated_time_features])

                # Update the input sequence by appending the new input
                next_input_tensor = torch.tensor(next_input, dtype=torch.float32).unsqueeze(0).unsqueeze(0).to(device)

                input_seq = torch.cat([
                    input_seq[:, 1:],  # Remove the first timestep
                    next_input_tensor  # Add the new timestep
                ], dim=1)
        return np.vstack(predicted_values)  # Combine predictions into a single array





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
        # QMessageBox.information(self.visualization, "Info.", comment)

    def load_model(self, model_path, input_size=8, hidden_size=128, num_layers=2, output_size=4):
        """
        Load a pre-trained model from a file.
        """
        # self.model = self.create_rnn_model(input_size, hidden_size, num_layers, output_size)
        # self.model.load_state_dict(torch.load(model_path))
        self.model = torch.load(model_path)
        # self.model.eval()


    def save_predictions_to_csv(self, predictions, file_path):
        """
        Save predictions to a CSV file.
        """
        predictions.to_csv(file_path, index=False)
        print(f"Predictions saved to {file_path}")



def save_original(data,split=0.2):
    base_dir = os.path.join(os.getcwd(), "Dataset")
    os.makedirs(base_dir,exist_ok=True)

    test_size = split
    train_size = 1 - test_size  # Remaining is training data
    split_index = int(len(data) * train_size)

    # Split manually
    train_data = data.iloc[:split_index]
    test_data = data.iloc[split_index:]

    headers = data.columns.tolist()

    pd.DataFrame(train_data).to_csv(
        os.path.join(base_dir, "train.txt"), index=False, header=headers
    )

    pd.DataFrame(test_data).to_csv(
        os.path.join(base_dir, "test.txt"), index=False, header=headers
    )

if __name__ == "__main__":
    model_training = ModelTraining()
    data_loader = CustomDataLoader()
    data_loader.load_data_from_txt(r"C:\Users\Pritam\Desktop\MTP_DATA\sensor_monitoring_gui\Data\output.txt")
    model_training.train_new_model(data_loader,epochs=500,split=0.2,learning_rate=0.01)

    data_loader.load_data_from_txt(r'C:\Users\Pritam\Desktop\MTP_DATA\sensor_monitoring_gui\Dataset\test.txt')


    model_training.load_model('best_model11.pth')

    y_true, y_pred = model_training.test_model(data_loader.data,prediction_steps=20)

    adj_r2_per_output = adjusted_r2_multioutput(y_true, y_pred, 4)
    mae_per_output = mean_absolute_error(y_true, y_pred, multioutput='raw_values')
    rmse_per_output = np.sqrt(mean_squared_error(y_true, y_pred, multioutput='raw_values'))

    print(f"MAE (per output): {mae_per_output}")
    print(f"RMSE (per output): {rmse_per_output}")
    print(f"Adjusted R2 (per output): {adj_r2_per_output}")

    base_dir = os.path.join(os.getcwd(), "Results")
    os.makedirs(base_dir,exist_ok=True)
    plot_training_results_loss(model_training)
    plot_training_results_accuracy(model_training)
    plot_pred_vs_true(y_true.values, y_pred.values)
    plot_residuals(y_true.values, y_pred.values)
