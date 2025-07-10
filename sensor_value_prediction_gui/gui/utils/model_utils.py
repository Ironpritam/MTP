# utils/model_utils.py
import joblib

def save_model(model, file_path):
    """
    Save the trained model to the specified path.

    :param model: The trained model to be saved
    :param file_path: Path to save the model file
    """
    joblib.dump(model, file_path)
    print(f"Model saved to {file_path}")

def load_model(file_path):
    """
    Load a model from the specified path.

    :param file_path: Path to load the model from
    :return: Loaded model
    """
    model = joblib.load(file_path)
    print(f"Model loaded from {file_path}")
    return model
