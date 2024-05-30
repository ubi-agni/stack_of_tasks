#!/usr/bin/env python3

from __future__ import annotations

import logging
from collections import namedtuple
from sys import argv

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication

import rospy

from stack_of_tasks.logger import fix_rospy_logging, sot_logger
from stack_of_tasks.ui.application import Logic_Main

logger = sot_logger.getChild("main")
from traits.trait_notifiers import set_ui_handler


class TraitsEvent(QtCore.QEvent):
    _QT_TRAITS_EVENT = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

    def __init__(self, handler, args, kwargs):
        super().__init__(TraitsEvent._QT_TRAITS_EVENT)
        self.handler = handler
        self.args = args
        self.kwargs = kwargs

    def __call__(self):
        self.handler(*self.args, **self.kwargs)


class EventProcessor(QtCore.QObject):
    def event(self, a0: TraitsEvent) -> bool:
        if a0.type() == TraitsEvent._QT_TRAITS_EVENT:
            a0()
            return True
        return super().event(a0)

    def postEvent(self, handler, *args, **kwargs):
        QApplication.instance().postEvent(self, TraitsEvent(handler, args, kwargs))


def main():
    rospy.init_node("ik")
    fix_rospy_logging(sot_logger)
    sot_logger.setLevel(logging.DEBUG)

    app = QApplication(argv)
    set_ui_handler(EventProcessor(app).postEvent)

    _ = Logic_Main()

    app.exec()


if __name__ == "__main__":
    main()
