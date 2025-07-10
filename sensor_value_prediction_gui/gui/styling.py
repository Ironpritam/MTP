# gui/styling.py
from PyQt5.QtGui import QFont, QColor

def get_font(size, bold=False):
    font = QFont("Arial", size)
    font.setBold(bold)
    return font

def get_primary_color():
    return QColor("#2E86C1")  # Blue theme

def get_secondary_color():
    return QColor("#AED6F1")  # Light blue theme

def apply_dark_theme(window):
    window.setStyleSheet("""
        QMainWindow {
            background-color: #2E3440;
            color: #ECEFF4;
        }
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
        QLabel {
            font-size: 18px;
            font-weight: bold;
        }
        QCheckBox {
            color: #ECEFF4;
        }
    """)


def apply_light_theme(window):
    window.setStyleSheet("""
        QMainWindow {
            background-color: #FFFFFF;
            color: #000000;
        }
        QPushButton {
            background-color: #E0E0E0;
            color: #000000;
            font-size: 14px;
            font-weight: bold;
            border-radius: 5px;
            padding: 10px;
        }
        QPushButton:hover {
            background-color: #D0D0D0;
        }
        QLabel {
            font-size: 18px;
            font-weight: bold;
        }
        QCheckBox {
            color: #000000;
        }
    """)
