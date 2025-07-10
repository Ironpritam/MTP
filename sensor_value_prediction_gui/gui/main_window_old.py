from PyQt5.QtWidgets import (
    QMainWindow, QPushButton, QVBoxLayout, QWidget, QFileDialog, QMessageBox,
    QLabel, QHBoxLayout, QCheckBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from gui.data_loader import DataLoader
from gui.model_training import ModelTraining
from gui.visualization import Visualization
from gui.styling import apply_dark_theme, apply_light_theme

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Monitoring Model Trainer")
        self.setGeometry(100, 100, 1920, 1080)
        self.setWindowState(Qt.WindowMaximized)

        # Apply dark theme initially
        apply_dark_theme(self)

        # Main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # Title Section
        title_layout = QHBoxLayout()
        title = QLabel("Sensor Monitoring and Model Training")
        title.setFont(QFont("Arial", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #88C0D0;")
        title_layout.addWidget(title)
        self.main_layout.addLayout(title_layout)

        # Theme Toggle
        self.theme_checkbox = QCheckBox("Enable Light Theme")
        self.theme_checkbox.setStyleSheet("color: #ECEFF4;")
        self.theme_checkbox.stateChanged.connect(self.toggle_theme)
        self.main_layout.addWidget(self.theme_checkbox)

        # Button Section
        button_layout = QVBoxLayout()
        self.load_button = QPushButton("Load CSV")
        self.load_button.clicked.connect(self.load_csv)
        button_layout.addWidget(self.load_button)

        self.train_button = QPushButton("Train Model")
        self.train_button.clicked.connect(self.train_model)
        button_layout.addWidget(self.train_button)

        self.fine_tune_button = QPushButton("Fine-Tune Model")
        self.fine_tune_button.clicked.connect(self.fine_tune_model)
        button_layout.addWidget(self.fine_tune_button)

        self.save_button = QPushButton("Save Model")
        self.save_button.clicked.connect(self.save_model)
        self.save_button.setEnabled(False)  # Initially disabled
        button_layout.addWidget(self.save_button)

        self.test_button = QPushButton("Test Model on Real-Time Data")
        self.test_button.clicked.connect(self.test_model)
        button_layout.addWidget(self.test_button)

        self.main_layout.addLayout(button_layout)

        # Visualization Section
        self.visualization = Visualization(self)
        self.main_layout.addWidget(self.visualization)

        # Initialize components
        self.data_loader = DataLoader()
        self.model_training = ModelTraining(self.visualization)
        self.model = None
        self.predictions = None

    def toggle_theme(self, state):
        if state == Qt.Checked:
            apply_light_theme(self)
        else:
            apply_dark_theme(self)

    def load_csv(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Data Files (*.csv *.xlsx *.xls)")
        if file_path:
            try:
                # Determine file type based on extension
                if file_path.endswith('.csv'):
                    self.data_loader.load_data(file_path)
                elif file_path.endswith(('.xlsx', '.xls')):
                    # Read Excel file and convert to CSV-like DataFrame
                    self.data_loader.load_data_from_excel(file_path)
                else:
                    raise ValueError("Unsupported file format")

                QMessageBox.information(self, "Data Loaded", "Data loaded successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file: {str(e)}")

    def train_model(self):
        if self.data_loader.data is not None:
            self.model = self.model_training.train_new_model(self.data_loader.data)
            self.save_button.setEnabled(True)  # Enable save button after training
        else:
            QMessageBox.warning(self, "Error", "Please load data first!")

    def fine_tune_model(self):
        if self.model is not None:
            self.model_training.fine_tune_model(self.model)
        else:
            QMessageBox.warning(self, "Error", "Please train a model first!")

    def test_model(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Real-Time Data CSV", "", "CSV Files (*.csv)")
        if file_path:
            real_time_data = self.data_loader.load_real_time_data(file_path)
            if real_time_data is not None:
                self.predictions = self.model_training.test_model(real_time_data)
                self.save_predictions()

    def save_model(self):
        if self.model is not None:
            save_path, _ = QFileDialog.getSaveFileName(self, "Save Model", "", "Model Files (*.model)")
            if save_path:
                self.model_training.save_model(self.model, save_path)
                QMessageBox.information(self, "Model Saved", "Model saved successfully!")

    def save_predictions(self):
        if self.predictions is not None:
            save_path, _ = QFileDialog.getSaveFileName(self, "Save Predictions", "", "CSV Files (*.csv)")
            if save_path:
                self.model_training.save_predictions_to_csv(self.predictions, save_path)
                QMessageBox.information(self, "Saved", "Predictions saved successfully!")
        else:
            QMessageBox.warning(self, "Error", "No predictions to save!")
