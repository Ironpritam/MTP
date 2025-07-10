# gui/main_window.py
from PyQt5.QtWidgets import (
    QMainWindow, QPushButton, QVBoxLayout, QWidget, QFileDialog, QMessageBox,
    QLabel, QHBoxLayout, QCheckBox, QProgressBar, QFrame, QInputDialog)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
import threading
from gui.data_loader import DataLoader
from gui.model_training import ModelTraining
from gui.visualization import Visualization
from gui.styling import apply_dark_theme, apply_light_theme

# class TrainingThread(QThread):
#     progress_signal = pyqtSignal(int)
#     task_finished = pyqtSignal(object)
#
#     def __init__(self, model_training, data):
#         super().__init__()
#         self.model_training = model_training
#         self.data = data
#
#     def run(self):
#         # Trigger the training process
#
#         self.model_training.train_new_model(self.data, epochs=10, progress_callback = self.update_progress)
#
#     def update_progress(self, progress):
#         self.progress_signal.emit(progress)





class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Monitoring Model Trainer")
        self.setGeometry(100, 100, 1280, 720)
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
        title.setFont(QFont("Arial", 24, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #88C0D0; margin-bottom: 20px;")
        title_layout.addWidget(title)
        self.main_layout.addLayout(title_layout)

        # Theme Toggle
        theme_layout = QHBoxLayout()
        theme_layout.setAlignment(Qt.AlignRight)
        self.theme_checkbox = QCheckBox("Enable Light Theme")
        self.theme_checkbox.setStyleSheet("""
            color: #ECEFF4;
            font-size: 16px;
            margin-right: 20px;
        """)
        self.theme_checkbox.stateChanged.connect(self.toggle_theme)
        theme_layout.addWidget(self.theme_checkbox)
        self.main_layout.addLayout(theme_layout)

        # Horizontal Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        self.main_layout.addWidget(separator)

        # Button Section
        button_layout = QVBoxLayout()
        button_layout.setSpacing(15)

        self.load_button = self.create_button("Load CSV")
        self.load_button.clicked.connect(self.load_csv)
        button_layout.addWidget(self.load_button)

        self.train_button = self.create_button("Train Model")
        self.train_button.clicked.connect(self.train_model)
        button_layout.addWidget(self.train_button)

        self.fine_tune_button = self.create_button("Fine-Tune Model")
        self.fine_tune_button.clicked.connect(self.fine_tune_model)
        button_layout.addWidget(self.fine_tune_button)

        self.save_button = self.create_button("Save Model", enabled=False)
        self.save_button.clicked.connect(self.save_model)
        button_layout.addWidget(self.save_button)

        self.test_button = self.create_button("Test Model on Real-Time Data")
        self.test_button.clicked.connect(self.test_model)
        button_layout.addWidget(self.test_button)

        self.main_layout.addLayout(button_layout)

        # Progress Bar
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                height: 20px;
                border-radius: 10px;
                text-align: center;
                background-color: #4C566A;
            }
            QProgressBar::chunk {
                background-color: #5E81AC;
                border-radius: 10px;
            }
        """)
        self.main_layout.addWidget(self.progress_bar)

        # Visualization Section
        self.visualization = Visualization(self)
        self.visualization.setStyleSheet("margin-top: 20px;")
        self.main_layout.addWidget(self.visualization)

        # Update text over graph plot for contrast
        self.update_graph_text_style()

        # Initialize components
        self.data_loader = DataLoader()
        self.model_training = ModelTraining(self.visualization)
        self.model = None
        self.predictions = None

        # Connect progress signal to update the progress bar
        self.model_training.progress_signal.connect(self.update_progress)

    def update_graph_text_style(self):
        """
        Updates the style of text displayed over the graph to ensure visibility
        on both dark and light themes.
        """
        # Adjust font and color for the visualization (graph plot)
        self.visualization.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: #ECEFF4; /* Default to light color for dark theme */
            }
        """)

        # If in light theme, use dark text for better contrast
        if self.theme_checkbox.isChecked():
            self.visualization.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    font-weight: bold;
                    color: #2E3440; /* Dark text for light theme */
                }
            """)

    def create_button(self, text, enabled=True):
        """Helper function to create styled buttons."""
        button = QPushButton(text)
        button.setEnabled(enabled)
        button.setStyleSheet("""
            QPushButton {
                background-color: #4C566A;
                color: #ECEFF4;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #5E81AC;
            }
            QPushButton:disabled {
                background-color: #3B4252;
                color: #7E8A99;
            }
        """)
        return button

    def toggle_theme(self, state):
        if state == Qt.Checked:
            apply_light_theme(self)
            self.update_graph_text_style()  # Update text style when switching theme
        else:
            apply_dark_theme(self)
            self.update_graph_text_style()  # Update text style when switching theme

    # Rest of your methods (load_csv, train_model, etc.) remain the same

    def load_csv(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Data Files (*.csv *.xlsx *.xls *.txt)")
        if file_path:
            try:
                # Determine file type based on extension
                if file_path.endswith('.csv'):
                    self.data_loader.load_data(file_path)
                elif file_path.endswith(('.xlsx', '.xls')):
                    # Read Excel file and convert to CSV-like DataFrame
                    self.data_loader.load_data_from_excel(file_path)
                elif file_path.endswith('.txt'):
                    self.data_loader.load_data_from_txt(file_path)
                else:
                    raise ValueError("Unsupported file format")
                if self.data_loader.data is not None:
                    QMessageBox.information(self, "Data Loaded", "Data loaded successfully!")
                else:
                    raise ValueError("Data Not loaded properly, try again!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file: {str(e)}")

    def train_model(self):
        if self.data_loader.data is not None:
            # Prompt user for number of epochs with default value
            epochs, ok_epochs = QInputDialog.getInt(
                self, "Set Training Epochs", "Enter the number of epochs (default: 10):", value=10, min=1, max=1000
            )
            if not ok_epochs:
                epochs = 10  # Set to default
                QMessageBox.information(
                    self, "Default Value Used", "Number of epochs not provided. Defaulting to 10."
                )

            # Prompt user for learning rate with default value
            learning_rate, ok_lr = QInputDialog.getDouble(
                self, "Set Learning Rate", "Enter the learning rate (default: 0.001):", value=0.001, min=1e-6, max=1.0, decimals=6
            )
            if not ok_lr:
                learning_rate = 0.001  # Set to default
                QMessageBox.information(
                    self, "Default Value Used", "Learning rate not provided. Defaulting to 0.001."
                )

            # Prompt user for split with default value
            split, ok_sp = QInputDialog.getDouble(
                self, "Set Learning Rate", "Enter the Train-Test split percentage (default: 0.2):", value=0.2, min=0, max=1.0, decimals=6
            )
            if not ok_sp:
                learning_rate = 0.2  # Set to default
                QMessageBox.information(
                    self, "Default Value Used", "Split percentage not provided. Defaulting to 0.2."
                )
                
            # Connect signals
            self.model_training.task_finished.connect(self.on_task_finished)
            self.model_training.progress_signal.connect(self.update_progress)

            # Start training with user-specified or default parameters
            self.model_training.train_new_model(self.data_loader.data, epochs=epochs, learning_rate=learning_rate, split=split)
        else:
            QMessageBox.warning(self, "Error", "Please load data first!")


    def on_task_finished(self, model):
        print("Task finished successfully!")  # Debugging message
        self.model = model
        self.save_button.setEnabled(True)
        QMessageBox.information(self, "Info", "Model has been trained! \nSave Model using 'Save Model' Button.")


    def cleanup_thread(self):
        # Properly stop and clean up the thread
        if self.model_training.isRunning():
            self.model_training.quit()  # Gracefully stop the thread
            self.model_training.wait()  # Wait for the thread to finish
        self.model_training.deleteLater()  # Safely delete the thread after it finishes

    def fine_tune_model(self):
        if self.model is not None:
            self.model_training.fine_tune_model(self.data_loader.data)
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
            save_path, _ = QFileDialog.getSaveFileName(self, "Save Model", "", "Model Files (*.pth)")
            if save_path:
                self.model_training.save_model(self.model, save_path)
                QMessageBox.information(self, "Model Saved", "Model saved successfully!")
        else:
            QMessageBox.warning(self, "Error", "No trained model to save!")

    def save_predictions(self):
        if self.predictions is not None:
            save_path, _ = QFileDialog.getSaveFileName(self, "Save Predictions", "", "CSV Files (*.csv)")
            if save_path:
                self.model_training.save_predictions_to_csv(self.predictions, save_path)
                QMessageBox.information(self, "Saved", "Predictions saved successfully!")
        else:
            QMessageBox.warning(self, "Error", "No predictions to save!")

    def update_progress(self, progress):
        """
        Update the progress bar with the current progress value.
        """
        self.progress_bar.setValue(progress)
