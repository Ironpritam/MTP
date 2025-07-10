# gui/visualization.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QComboBox, QLabel
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

class Visualization(QWidget):
    def __init__(self, parent=None):
        super(Visualization, self).__init__(parent)

        # Initialize figure and canvas
        self.figure, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)

        # Dropdown for selecting parameters to plot
        self.parameter_selector = QComboBox()
        self.parameter_selector.addItems(["CO2", "Temperature", "Humidity"])
        self.parameter_selector.currentIndexChanged.connect(self.update_plot)

        # Label for the dropdown
        self.selector_label = QLabel("Select Parameter to Plot:")

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.selector_label)
        layout.addWidget(self.parameter_selector)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Store data for plotting
        self.true_values = None
        self.predicted_values = None
        self.parameters = ["CO2", "Temperature", "Humidity"]

    def plot_training_results(self, env, model):
        """
        Plots true vs. predicted values for training.
        """
        self.ax.clear()
        self.true_values = env.data.iloc[env.history_length:][self.parameters].values
        self.predicted_values = []

        # Generate predictions
        obs = env.reset()
        for _ in range(len(env.data) - env.history_length):
            action, _ = model.predict(obs)
            self.predicted_values.append(action)
            obs, _, done, _ = env.step(action)
            if done:
                break

        self.predicted_values = np.array(self.predicted_values)
        self.update_plot()

    def plot_real_time_results(self, true_data, predicted_data):
        """
        Plots true vs. predicted values for real-time data.
        """
        self.ax.clear()
        self.true_values = true_data[self.parameters].values
        self.predicted_values = predicted_data
        self.update_plot()

    def update_plot(self):
        """
        Updates the plot based on the selected parameter.
        """
        if self.true_values is None or self.predicted_values is None:
            return

        # Clear the axes
        self.ax.clear()

        # Get the selected parameter
        parameter = self.parameter_selector.currentText()
        index = self.parameters.index(parameter)

        # Plot true and predicted values
        self.ax.plot(self.true_values[:, index], label=f"True {parameter}", color='blue')
        self.ax.plot(self.predicted_values[:, index], label=f"Predicted {parameter}", linestyle='--', color='orange')

        # Add legend and labels
        self.ax.set_title(f"{parameter}: True vs Predicted")
        self.ax.set_xlabel("Time Steps")
        self.ax.set_ylabel(parameter)
        self.ax.legend()

        # Redraw the canvas
        self.canvas.draw()
