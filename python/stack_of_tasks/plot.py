#!/usr/bin/env python
import typing
import sys
import pyqtgraph as pg
from pyqtgraph.dockarea import Dock,DockArea
import numpy as np

from PyQt5.QtCore import QSize, QTimer

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QMainWindow
import sys
import timeit

pg.setConfigOption('enableExperimental', True)

class MainWindow(QMainWindow):
    def __init__(self, number_of_plots):
        super().__init__()

        self.setWindowTitle("My App")
        self.nop = number_of_plots
        cw = QWidget()
        l = QVBoxLayout()
        cw.setLayout(l)

        self.data = np.zeros((number_of_plots, 0))
        self.plots = []
        
        for x in range(number_of_plots):
            p = pg.PlotWidget()
            p.addItem(pg.PlotItem())

            if x > 0:
                p.setXLink(self.plots[x-1])
            self.plots.append(p)
            l.addWidget(p)

        self.setCentralWidget(cw)


        self.timer = QTimer()
        self.timer.timeout.connect(self.new_data)
        self.timer.start(30)

    def new_data(self):
        self.data = np.c_[self.data, np.random.uniform(size=(self.nop,1))]
        for x in range(self.nop):
            self.plots[x].plot(self.data[x,:], clear=True)



def main():
    app = QApplication(sys.argv)
    window = MainWindow(4)
    window.show()

    app.exec()


if __name__ == '__main__':

    main()

