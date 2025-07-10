# gui/data_loader.py
import pandas as pd
from sklearn.preprocessing import MinMaxScaler

class DataLoader:
    def __init__(self):
        self.data = None
        self.scaler = MinMaxScaler()  # Keep a reference to reuse the scaler
        self.sensor_columns = [
                    'CH1', 'CH2', 'CH3',
                    'hour', 'day_of_week', 'is_weekend'
                ]

    def _detect_header(self, file_path):
        """
        Detect if the first row is a header by analyzing the content.
        """
        try:
            with open(file_path, 'r') as file:
                first_line = file.readline()
                second_line = file.readline()

            # Split lines into values
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

            # Assign default column names if headers are not present
            if header is None:
                self.data.columns = ['Timestamp'] + [f'Feature_{i}' for i in range(1, len(self.data.columns))]

            # Attempt to detect timestamp format in the 'Timestamp' column
            if 'Timestamp' in self.data.columns:
                try:
                    self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'], unit='s')  # Try UNIX timestamp
                except Exception:
                    try:
                        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'], format='%Y-%m-%d %H:%M:%S')
                    except Exception:
                        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'])  # Fallback to auto-detection

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
                data.columns = ['Timestamp'] + [f'Feature_{i}' for i in range(1, len(data.columns))]

            # Attempt to detect timestamp format in the first column
            try:
                data['Timestamp'] = pd.to_datetime(data['Timestamp'], unit='s')  # Try UNIX timestamp
            except Exception:
                try:
                    data['Timestamp'] = pd.to_datetime(data['Timestamp'], format='%Y-%m-%d %H:%M:%S')  # Try standard datetime format
                except Exception:
                    data['Timestamp'] = pd.to_datetime(data['Timestamp'])  # Fallback to auto-detection

            self.data = data
            self._preprocess_data()
        except Exception as e:
            print(f"An error occurred while loading the data: {e}")
            self.data = None


    def load_data_from_excel(self, file_path, sheet_name='Data In'):
        """
        Load and preprocess data from an Excel file.
        """
        try:
            # Load the data from the specified sheet
            raw_data = pd.read_excel(file_path, sheet_name=sheet_name, skiprows=6, header=None)

            # Assign default column names if headers are not present
            if not raw_data.columns[0].startswith('TIME'):
                raw_data.columns = ['TIME'] + [f'Feature_{i}' for i in range(1, len(raw_data.columns))]

            # Rename 'TIME' to 'Timestamp'
            raw_data = raw_data.rename(columns={'TIME': 'Timestamp'})

            # Parse 'Timestamp' as datetime
            try:
                raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'], unit='s')  # Try UNIX timestamp
            except Exception:
                try:
                    raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'], format='%Y-%m-%d %H:%M:%S')
                except Exception:
                    raw_data['Timestamp'] = pd.to_datetime(raw_data['Timestamp'])  # Fallback to auto-detection

            columns_to_drop = [col for col in raw_data.columns if col not in self.sensor_columns and col != 'Timestamp']
            raw_data = raw_data.drop(columns=columns_to_drop)

            # Drop any rows with NaN in the Timestamp (if any)
            raw_data = raw_data.dropna(subset=['Timestamp'])

            # Assign cleaned data to self.data for preprocessing
            self.data = raw_data
            self._preprocess_data()
        except Exception as e:
            print(f"Error loading data from Excel: {e}")
            self.data = None


    def _preprocess_data(self):
        """
        Preprocess the loaded data: add features, clean data, and normalize.
        """
        if self.data is None:
            return

        # Convert all non-Timestamp columns to numeric in a vectorized way
        self.data.loc[:, self.data.columns != 'Timestamp'] = self.data.loc[:, self.data.columns != 'Timestamp'].apply(
            pd.to_numeric, errors='coerce'
        )

        # Drop rows with any NaN values across the entire dataframe
        self.data = self.data.dropna()

        # Feature Engineering: Add new time-based features
        self.data.loc[:, 'hour'] = self.data['Timestamp'].dt.hour
        self.data.loc[:, 'day_of_week'] = self.data['Timestamp'].dt.dayofweek
        self.data.loc[:, 'is_weekend'] = self.data['day_of_week'].apply(lambda x: 1 if x >= 5 else 0)

        # Removing Timestamp column to match dimension
        if 'Timestamp' in self.data.columns:
            self.data = self.data.drop(columns=['Timestamp'])


        # Columns to normalize
        # self.sensor_columns = [
        #     'Temperature(in oC)', 'Humidity(in RH)', 'CO2(in ppm)', 'PN1(in µg/m³)',
        #     'PN2.5', 'PN4', 'PN10', 'Oxygen(%)', 'Ozone(analog_value)',
        #     'MICS_Concentration(analog_value)', 'hour', 'day_of_week', 'is_weekend'
        # ]


        # Check for missing columns
        missing_cols = [col for col in self.sensor_columns if col not in self.data.columns]
        if missing_cols:
            print(f"Warning: Missing columns in data: {missing_cols}")

        # Normalize existing sensor columns using .loc to avoid SettingWithCopyWarning
        existing_cols = [col for col in self.sensor_columns if col in self.data.columns]
        self.data.loc[:, existing_cols] = self.scaler.fit_transform(self.data.loc[:, existing_cols])



    def _preprocess_real_time_data(self, real_time_data):
        """
        Preprocess real-time data for testing.
        """
        if real_time_data is None:
            return None

        # Feature Engineering for real-time data
        real_time_data['hour'] = real_time_data['Timestamp'].dt.hour
        real_time_data['day_of_week'] = real_time_data['Timestamp'].dt.dayofweek
        real_time_data['is_weekend'] = real_time_data['day_of_week'].apply(lambda x: 1 if x >= 5 else 0)



        # Normalize using the same scaler
        existing_cols = [col for col in self.sensor_columns if col in real_time_data.columns]
        real_time_data[existing_cols] = self.scaler.transform(real_time_data[existing_cols])

        return real_time_data
