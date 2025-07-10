# utils/file_manager.py
import pandas as pd

def save_predictions_to_csv(predictions, file_path):
    predictions.to_csv(file_path, index=False)
