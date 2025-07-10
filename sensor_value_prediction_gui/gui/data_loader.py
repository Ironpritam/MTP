# gui/data_loader.py
import pandas as pd
from sklearn.preprocessing import MinMaxScaler, StandardScaler
from torch.utils.data import Dataset
import numpy as np
from PyQt5.QtWidgets import QMessageBox
import os


pd.set_option('future.no_silent_downcasting', True)

def update_time_features(last_known_time_features, step_minutes=1):
    """
    Update time-related features (hour, minute, day_of_week, is_weekend) for the next step.

    :param last_known_time_features: Array of time-based features from the last known input.
    :param step_minutes: Number of minutes to increment for each prediction step.
    :return: Updated array of time-based features.
    """
    # Extract individual components
    hour, minute, day_of_week, is_weekend = last_known_time_features

    # Increment minute
    minute += step_minutes
    if minute >= 60:
        minute %= 60
        hour += 1  # Increment hour when minutes overflow

    # Handle hour overflow
    if hour >= 24:
        hour %= 24
        day_of_week = (day_of_week + 1) % 7  # Increment day of the week
        is_weekend = 1 if day_of_week >= 5 else 0  # Update weekend flag

    return np.array([hour, minute, day_of_week, is_weekend], dtype=np.float32)



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
        os.path.join(base_dir, "train.csv"), index=False, header=headers
    )

    pd.DataFrame(test_data).to_csv(
        os.path.join(base_dir, "test.csv"), index=False, header=headers
    )


def get_original_back(X, y, pred_sensors_count=4, scaler=None):
    """
    Reconstruct the original dataset from X and y.
    """
    if scaler:
        # Apply inverse transformation to sensor columns
        X[:, :, :pred_sensors_count] = scaler.inverse_transform(X[:, :, :pred_sensors_count].reshape(-1, pred_sensors_count)).reshape(X.shape[0], X.shape[1], pred_sensors_count)

    # Step 1: Collect the first rows from all windows in X except the last one
    reconstructed = [x_window[0] for x_window in X[:-1]]

    # Step 2: Add all rows from the last window in X
    reconstructed.extend(X[-1])

    # Step 3: Append the last row of y and update time-based features
    updated_time_fs = update_time_features(X[-1, -1, pred_sensors_count:])
    reconstructed.append(np.concatenate([y[-1], updated_time_fs]))

    # Convert to a numpy array
    reconstructed = np.array(reconstructed, dtype=np.float32)
    return reconstructed


def save_splits(X_train, X_test, y_train, y_test):
    """
    Save train and test splits into separate directories.
    """
    # Define directories
    base_dir = os.path.join(os.getcwd(), "Dataset")
    train_dir = os.path.join(base_dir, "train")
    test_dir = os.path.join(base_dir, "test")

    # Create directories if they don't exist
    os.makedirs(base_dir,exist_ok=True)
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)

    try:
        # Save train data
        # train = get_original_back(X_train, y_train, len(y_train[0]), True)
        # pd.DataFrame(train).to_csv(
        #     os.path.join(train_dir, "train.csv"), index=False, header=False
        # )
        pd.DataFrame(X_train.reshape(X_train.shape[0], -1)).to_csv(
            os.path.join(train_dir, "X_train.csv"), index=False, header=False
        )
        pd.DataFrame(y_train).to_csv(
            os.path.join(train_dir, "y_train.csv"), index=False, header=False
        )



        # Save test data
        # test = get_original_back(X_test,y_test,len(y_test[0]))
        # pd.DataFrame(test).to_csv(
        #     os.path.join(test_dir, "test.csv"), index=False, header=False
        # )
        pd.DataFrame(X_test.reshape(X_test.shape[0], -1)).to_csv(
            os.path.join(test_dir, "X_test.csv"), index=False, header=False
        )
        pd.DataFrame(y_test).to_csv(
            os.path.join(test_dir, "y_test.csv"), index=False, header=False
        )

        print(f"Train and test splits saved in '{base_dir}'.")
        # QMessageBox.information(self.visualization, "Data Saved", f"Train and test data saved in '{base_dir}'.")
    except Exception as e:
        error_msg = f"Failed to save train/test splits: {str(e)}"
        print(error_msg)
        # QMessageBox.critical(self.visualization, "Error", error_msg)


class DataLoader:
    def __init__(self):
        self.data = None
        # self.xscaler = MinMaxScaler()  # Reuse scaler for normalization
        # self.yscaler = MinMaxScaler()
        # self.sensor_columns = [
        #     'Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)'  ]
        self.sensor_columns = [
            'Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)',
            'Temperature(in oC)_Rolling_Mean', 'Temperature(in oC)_Rolling_Std', 'Temperature(in oC)_Rate_of_Change', 'Temperature(in oC)_Detrended',
            'Humidity(in RH)_Rolling_Mean', 'Humidity(in RH)_Rolling_Std', 'Humidity(in RH)_Rate_of_Change', 'Humidity(in RH)_Detrended',
            'CO2(in ppm)_Rolling_Mean', 'CO2(in ppm)_Rolling_Std', 'CO2(in ppm)_Rate_of_Change', 'CO2(in ppm)_Detrended',
            'Oxygen(%)_Rolling_Mean', 'Oxygen(%)_Rolling_Std', 'Oxygen(%)_Rate_of_Change', 'Oxygen(%)_Detrended'
        ]


        self.all_headers = ['Timestamp','Temperature(in oC)','Humidity(in RH)','CO2(in ppm)','PN1(in µg/m³)','PN2.5','PN4','PN10','Oxygen(%)','Ozone(analog_value)','MICS_Concentration(analog_value)']

    def _detect_header(self, file_path):
        """
        Detect if the first row is a header by analyzing the content.
        """
        try:
            with open(file_path, 'r') as file:
                first_line = file.readline()
                second_line = file.readline()

            first_line_values = first_line.strip().split(',')
            second_line_values = second_line.strip().split(',')

            # Check if the first line contains mostly non-numeric values
            is_header = all(not value.replace('.', '', 1).isdigit() for value in first_line_values)

            return 0 if is_header else None
        except Exception as e:
            print(f"Error detecting header: {e}")
            return None

    def load_data(self, file_path):
        """
        Load and preprocess data from a CSV file.
        """
        try:
            # Detect if the first row is a header
            header = self._detect_header(file_path)

            # Load the data
            self.data = pd.read_csv(file_path, header=header)

            if header is None:
                self.data.columns = [f'{self.all_headers[i]}' for i in range(0, len(self.data.columns))]

            # Try to detect and convert the 'Timestamp' column
            if 'Timestamp' in self.data.columns:
                try:
                    self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'], unit='s')  # Try UNIX timestamp
                except Exception:
                    try:
                        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'], format='%Y-%m-%d %H:%M:%S')
                    except Exception:
                        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'])  # Fallback

            columns_to_drop = [col for col in self.data.columns if col not in self.sensor_columns and col != 'Timestamp']
            self.data.drop(columns=columns_to_drop,inplace=True)
            self._preprocess_data()
        except Exception as e:
            print(f"Error loading data: {e}")
            self.data = None


    def load_data_from_txt(self, file_path):
        """
        Load data from a text file with comma-separated values.

        The first column is assumed to be a timestamp, which will be automatically detected and converted.
        """
        try:
            # Detect if the first row is a header
            header = self._detect_header(file_path)

            # Read the data from the file
            data = pd.read_csv(file_path, header=header)

            # Assign default column names if headers are not present
            if header is None:
                data.columns = [f'{self.all_headers[i]}' for i in range(0, len(data.columns))]

            # Attempt to detect timestamp format in the first column
            try:
                data['Timestamp'] = pd.to_datetime(data['Timestamp'], unit='s')  # Try UNIX timestamp
            except Exception:
                try:
                    data['Timestamp'] = pd.to_datetime(data['Timestamp'], format='%Y-%m-%d %H:%M:%S')  # Try standard datetime format
                except Exception:
                    data['Timestamp'] = pd.to_datetime(data['Timestamp'])  # Fallback to auto-detection

            columns_to_drop = [col for col in data.columns if col not in self.sensor_columns and col != 'Timestamp']
            data.drop(columns=columns_to_drop,inplace=True)
            self.data = data
            # /// cutting preprocessing , do it afterloading data its necessary !
            # self._preprocess_data()
        except Exception as e:
            print(f"An error occurred while loading the data: {e}")
            self.data = None


    def load_data_from_excel(self, file_path, sheet_name='Data In'):
        """
        Load and preprocess data from an Excel file.
        """
        try:
            raw_data = pd.read_excel(file_path, sheet_name=sheet_name, skiprows=6, header=None)

            if not raw_data.columns[0].startswith('TIME'):
                raw_data.columns = [f'{self.all_headers[i]}' for i in range(0, len(raw_data.columns))]

            raw_data = raw_data.rename(columns={'TIME': 'Timestamp'})

            try:
                raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'], unit='s')
            except Exception:
                try:
                    raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'], format='%Y-%m-%d %H:%M:%S')
                except Exception:
                    raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'])

            columns_to_drop = [col for col in raw_data.columns if col not in self.sensor_columns and col != 'Timestamp']
            raw_data.drop(columns=columns_to_drop,inplace=True)

            self.data = raw_data
            # /// cutting preprocessing , do it afterloading data its necessary !
            # self._preprocess_data()
        except Exception as e:
            print(f"Error loading data from Excel: {e}")
            self.data = None


    def _preprocess_data(self):
        """
        Preprocess the loaded data: handle features, missing data, and normalization.
        """
        if self.data is None:
            return

        # Ensure all columns except 'Timestamp' are numeric
        self.data.loc[:, self.data.columns != 'Timestamp'] = self.data.loc[:, self.data.columns != 'Timestamp'].apply(
            pd.to_numeric, errors='coerce'
        )

        # Drop rows with NaN values
        self.data = self.data.dropna().reset_index(drop=True)



        # Drop the 'Timestamp' column
        #   /// commenting to transfer this part in model_training to save test.csv and train.csv files as original as possible

        # if 'Timestamp' in self.data.columns:
        #     self.data = self.data.drop(columns=['Timestamp'])

        # missing_cols = [col for col in self.sensor_columns if col not in self.data.columns]
        # if missing_cols:
        #     print(f"Warning: Missing columns in data: {missing_cols}")


        # /// Trying Feature Engineering
        window_size = 20  # Example: 60-minute rolling window

        for sensor in ['Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'Oxygen(%)']:
            # Rolling Mean
            self.data[f'{sensor}_Rolling_Mean'] = self.data[sensor].rolling(window=window_size).mean()

            # Rolling Standard Deviation
            self.data[f'{sensor}_Rolling_Std'] = self.data[sensor].rolling(window=window_size).std()

            # Rate of Change
            self.data[f'{sensor}_Rate_of_Change'] = self.data[sensor].diff()

            # Detrended Values
            self.data[f'{sensor}_Detrended'] = self.data[sensor] - self.data[f'{sensor}_Rolling_Mean']

        # Fill NaN values introduced by rolling operations
        self.data = self.data.bfill().infer_objects(copy=False)

        self.scalers = {}
        # Normalize each column in the DataFrame
        for column in self.data.columns.difference(['Timestamp']):
            scaler = MinMaxScaler()
            self.data[column] = scaler.fit_transform(self.data[[column]])
            self.scalers[column] = scaler

        # Feature Engineering: Add detailed time-based features
        self.data = self.data.assign(
            hour=self.data['Timestamp'].dt.hour,
            minute=self.data['Timestamp'].dt.minute,
            # second=self.data['Timestamp'].dt.second,
            day_of_week=self.data['Timestamp'].dt.dayofweek,
            is_weekend=self.data['Timestamp'].dt.dayofweek.apply(lambda x: 1 if x >= 5 else 0),
            # part_of_day=self.data['Timestamp'].dt.hour.apply(self._determine_part_of_day)
        )

        # /// End FE

        # existing_cols = [col for col in self.sensor_columns if col in self.data.columns]
        # self.data[existing_cols] = self.scaler.fit_transform(self.data[existing_cols])

        with open('processed.csv', mode='w') as text_file:
            # Write the data to the text file as comma-separated values
            self.data.to_csv(text_file, index=False, header=True)



    def _prepare_time_series_data(self, input_length, output_length,pred_sensors_count):
        """
        Prepare time-series data for GRU training.
        """

        features = self.data.values


        X, y = [], []

        for i in range(features.shape[0] - input_length - output_length + 1):
            X.append(features[i:i + input_length])  # Input sequence
            y.append(features[i + input_length, 0:pred_sensors_count])  # Output (next steps)

        # X = np.array(X)
        # # X = self.scaler.fit_transform(X.reshape(-1, X.shape[-1])).reshape(X.shape)
        # X_normalize = X[:,:, 0:4]
        # X_normalized = self.xscaler.fit_transform(X_normalize.reshape(-1, X_normalize.shape[-1])).reshape(X_normalize.shape)

        # Combine normalized columns with the rest of the dataset
        # X[:,:, 0:4] = X_normalized
        y = np.array(y, dtype=np.float32)
        # y = self.yscaler.fit_transform(y)
        X = np.array(X,dtype=np.float32)
        return X, y

    def prepare_datasets(self, input_length, output_length,pred_sensors_count):
        """
        Prepare datasets for training and testing.
        """
        X, y = self._prepare_time_series_data(input_length, output_length,pred_sensors_count)
        # print(type(X),"\n",type(y))
        return X, y

    def _preprocess_real_time_data(self, real_time_data):
        """
        Preprocess real-time data for testing.
        """
        if real_time_data is None:
            return None

        real_time_data['hour'] = real_time_data['Timestamp'].dt.hour
        real_time_data['day_of_week'] = real_time_data['Timestamp'].dt.dayofweek
        real_time_data['is_weekend'] = real_time_data['day_of_week'].apply(lambda x: 1 if x >= 5 else 0)

        existing_cols = [col for col in self.sensor_columns if col in real_time_data.columns]
        real_time_data[existing_cols] = self.scaler.transform(real_time_data[existing_cols])

        return real_time_data
