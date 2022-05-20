from PyQt5 import QtCore, QtWidgets


class Collapse(QtWidgets.QWidget):
    def __init__(self, parent=None, title="Title") -> None:
        super().__init__(parent)


        self.mainLayout = QtWidgets.QVBoxLayout()
        self.mainLayout.setSpacing(0)
        self.mainLayout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.mainLayout)

        self.toggle_button = QtWidgets.QToolButton(
            text=title, checkable=True, checked=False
        )
        self.toggle_button.setToolButtonStyle(
            QtCore.Qt.ToolButtonTextBesideIcon
        )
        self.toggle_button.setStyleSheet("border: none;")
        self.toggle_button.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        self.toggle_button.setArrowType(QtCore.Qt.RightArrow)
        self.toggle_button.clicked.connect(self.on_clicked)

        self.buttonLayout = QtWidgets.QHBoxLayout()
        self.buttonLayout.setContentsMargins(0,0,0,0)
        self.buttonLayout.addWidget(self.toggle_button)

        self.mainLayout.addLayout(self.buttonLayout)
        self.content = QtWidgets.QWidget()

        self._contentLayout = None
        
        self.content.setMaximumHeight(0)
        self.mainLayout.addWidget(self.content)

        #self.contentScrollArea = QtWidgets.QScrollArea(maximumHeight=0, minimumHeight=0)
        #self.contentScrollArea.setSizePolicy(
        #    QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        #)
        #self.mainLayout.addWidget(self.contentScrollArea)
#
        #self.toggle_animation = QtCore.QParallelAnimationGroup(self)
        #self.toggle_animation.addAnimation(
        #    QtCore.QPropertyAnimation(self, b"minimumHeight")
        #)
        #self.toggle_animation.addAnimation(
        #    QtCore.QPropertyAnimation(self, b"maximumHeight")
        #)
        #self.toggle_animation.addAnimation(
        #    QtCore.QPropertyAnimation(self.contentScrollArea, b"maximumHeight")
        #)


    def sizeHint(self) -> QtCore.QSize:
        isOpen = self.toggle_button.isChecked()
        h = self._contentLayout.sizeHint().height() if isOpen and self._contentLayout else 0
        self.content.setMinimumHeight(h)
        self.content.setMaximumHeight(h)

        return super().sizeHint()

    @QtCore.pyqtSlot()
    def on_clicked(self):
        isOpen = self.toggle_button.isChecked()
        self.toggle_button.setArrowType(
            QtCore.Qt.DownArrow if isOpen else QtCore.Qt.RightArrow
        )
        self.updateGeometry()


class CollapseWithDelete(Collapse):

    delete_self = QtCore.pyqtSignal()

    def __init__(self, parent=None, title="Title") -> None:
        super().__init__(parent, title)
        self.deleteButton = QtWidgets.QToolButton()
        icon = self.style().standardIcon(QtWidgets.QStyle.SP_TitleBarCloseButton)
        self.deleteButton.setIcon(icon)
        self.deleteButton.setToolButtonStyle(
            QtCore.Qt.ToolButtonIconOnly
        )
        self.deleteButton.setStyleSheet("border: none;")
        self.buttonLayout.addWidget(self.deleteButton)
        self.deleteButton.clicked.connect(lambda _: self.delete_self.emit())
