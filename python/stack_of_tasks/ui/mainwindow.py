from PyQt5 import QtWidgets
from .generated.MainWindow import Ui_MainWindow


class Ui(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        self.setupUi(self)
        
        self.show()

