#!/usr/bin/env python3

import sys
import signal
from threading import Lock, Thread

import numpy
import rospy
from PyQt5.QtWidgets import QApplication
from tf import transformations as tf

from stack_of_tasks.controller import Controller, MarkerControl
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.utils import MARKERS, TASKS


class Main:
    def __init__(self) -> None:
        # Controller Setup
        self.controller = Controller(rate=50)
        self.marker = MarkerControl()
        self.ui = Ui()

        self.targets = {}

        self.task_hierarchy = self.controller.task_hierarchy

        self.control_is_running = False
        self._thread = None

        # Controller callbacks

        self.controller.T_callback.append(lambda T: self.set_target_data("T", T))
        self.controller.J_callback.append(lambda J: self.set_target_data("J", J))
        self.controller.joint_state_callback.append(
            lambda jp: self.set_target_data("joint_position", jp)
        )

        self.set_target_data("min", numpy.atleast_2d(self.controller.mins))
        self.set_target_data("max", numpy.atleast_2d(self.controller.maxs))

        # UI Callbacks

        self.ui.marker.set_available_marker_classes(MARKERS.keys())

        # hierarchy callbacks
        self.ui.hierarchy.new_task_callback = self._new_task_from_classname

        self.ui.hierarchy.new_task.connect(self._new_task_from_classname)
        self.ui.hierarchy.move_level_signal.connect(self._swap_tasks)
        self.ui.hierarchy.move_task_signal.connect(self._move_task)
        self.ui.hierarchy.delete_level_signal.connect(self._delete_level)
        self.ui.hierarchy.delete_task_signal.connect(self._delete_task)

        # Marker callback
        self.ui.marker.new_marker_signal.connect(self._create_marker)
        self.marker.marker_data_callback.append(self.set_target_data)

        # Target Callbacks

        self.ui.targets.value_changed.connect(
            lambda name, value: self.set_target_data(name, value, False)
        )

        # Commands
        self.ui.toggleRun.clicked.connect(self.start_hierarchy_control)
        # self.ui.toggleRun.clicked.connect(self.debug)
        self.ui.adjustSize()
        self.controller.reset()

    def on_quit(self):
        if self.control_is_running:
            self.control_is_running = False
            self._thread.join()

    # marker operations

    def _create_marker(self, class_name, data):
        pose = tf.translation_matrix(data["pos"].T)
        pose = pose.dot(tf.euler_matrix(*data["rot"]))

        m = MARKERS[class_name]["class"](name=data["name"], pose=pose, scale=data["scale"])
        self.marker.add_marker(m, data["name"])
        self.ui.marker.add_marker(data["name"])

    # hierarchy operations

    def _new_task_from_classname(self, task_name: str, level: int):
        # create instance
        task_class = TASKS[task_name]["class"]
        task = task_class()

        if level == -1:
            self.task_hierarchy.add_task_lower(task)
            level = self.task_hierarchy.higest_hiracy_level
        else:
            self.task_hierarchy.add_task_at(task, level)

        self.ui.hierarchy.add_task_at_level(task, level, list(self.targets.keys()))

    def _swap_tasks(self, a, b):
        l = self.task_hierarchy.hierarchy.pop(a)
        self.task_hierarchy.hierarchy.insert(b, l)

    def _move_task(self, task, source, target):
        if target == -1:
            pass
        else:
            self.task_hierarchy.hierarchy[source].remove(task)
            self.task_hierarchy.hierarchy[target].append(task)

            if not self.task_hierarchy.hierarchy[source]:
                del self.task_hierarchy.hierarchy[source]
                self.ui.hierarchy.delete_level(source)

    def _delete_task(self, level, task):
        self.task_hierarchy.hierarchy[level].remove(task)
        if not self.task_hierarchy.hierarchy[level]:
            self.ui.hierarchy.delete_level(level)
            self._delete_level(level)

    def _delete_level(self, index):
        del self.task_hierarchy.hierarchy[index]

    def print_hierarchy(self):
        print("hierarchy:")
        for i, l in enumerate(self.task_hierarchy.hierarchy):
            print(f"Level {i}")
            for t in l:
                print(f"    {t.name}")
                print(f"        {t.argmap}")

    # target operation

    def set_target_data(self, name, data, notify_ui=True):
        self.targets[name] = data
        self.ui.hierarchy.target_data_changed.emit(list(self.targets.keys()))
        if notify_ui:
            self.ui.targets.set_target_value(name, data)

    # commands
    def set_hierarchy_editable(self, editable):
        self.ui.hierarchy.setEnabled(editable)

    def debug(self):
        self.print_hierarchy()

    def start_hierarchy_control(self):
        if self.control_is_running:
            self.control_is_running = False
            self.set_hierarchy_editable(True)
            self._thread.join()
            self.ui.toggleRun.setText("Run")
        else:
            self.control_is_running = True
            self.set_hierarchy_editable(False)

            def run_func():
                rate = rospy.Rate(50)
                while self.control_is_running:
                    self.controller.hierarchic_control(self.targets)

                    rate.sleep()

            self._thread = Thread(target=run_func, daemon=True)
            self._thread.start()
            self.ui.toggleRun.setText("Stop")


if __name__ == "__main__":
    rospy.init_node("ik")

    app = QApplication(["TH", *sys.argv])

    main = Main()
    app.aboutToQuit.connect(main.on_quit)

    # install the default interrupt handler for Ctrl-C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app.exec()
