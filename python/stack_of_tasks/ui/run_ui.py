#!/usr/bin/env python3

import sys

import numpy as np
from PyQt5.QtCore import QMetaObject, QObject, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from stack_of_tasks.controller import MarkerControl
from stack_of_tasks.marker.markers import SixDOFMarker
from stack_of_tasks.ref_frame.frames import JointFrame
from stack_of_tasks.ref_frame.frames.World import Offset, World
from stack_of_tasks.robot_model import JointStatePublisher, RobotModel, RobotState
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.model.av_ref import AvailableRefModel


class MarkerController:
    def __init__(self) -> None:
        self.marker_server = InteractiveMarkerServer("controller")
        self.maker = []


class Ui_Controller(QObject):

    instance = None

    @staticmethod
    def get_instance():
        return Ui_Controller.instance

    def __init__(self) -> None:
        super().__init__()

        Ui_Controller.instance = self

        self.mc = MarkerControl()

        self.hierarchy = TaskHierarchy()
        self.robot_model = RobotModel()
        self.robot_state = RobotState(self.robot_model)

        self.jsp = JointStatePublisher(self.robot_state)

        self.ref_model = AvailableRefModel()
        self._init_data()

        self.solver = OSQPSolver(self.robot_model.N, self.hierarchy, rho=0.01)

        self._solver_thread = QThread(self)
        self._solver_thread.setObjectName("main")
        self._solver_thread.start()

        self._solver_worker = SolverWorker(self, 50)
        self._solver_worker.moveToThread(self._solver_thread)

        self.ui = Ui(self.ref_model, self.hierarchy)
        self.ui.run_Button.clicked.connect(self.run_triggered)
        self.ui.destroyed.connect(self._solver_thread.terminate)

        self.ui.show()

    def _init_data(self):
        self.ref_model.add_ref(World(), "World")
        x = Offset(World()).rotate(x=0, y=-0.383, z=0, w=0.924).translate(x=0.5, z=0.5)
        self.ref_model.add_ref(x, "Target")

        self.s = SixDOFMarker(frame=x, scale=0.25)
        self.mc.add_marker(self.s, "")

        f = None
        for n in self.robot_model.links.keys():
            jf = JointFrame(self.robot_state, n)
            if n == "panda_hand_tcp":
                f = jf
            self.ref_model.add_ref(jf, n)

        from stack_of_tasks.tasks.Eq_Tasks import PositionTask
        from stack_of_tasks.tasks.Task import TaskSoftnessType

        print(x, f)
        t = PositionTask(x, f, softness=TaskSoftnessType.linear, weight=1)
        self.hierarchy.append_task(t)

    def run_triggered(self):
        if self._solver_worker.running:
            self._solver_worker.stop()
        else:
            QMetaObject.invokeMethod(self._solver_worker, "start")

    @pyqtSlot()
    def save_exit(self):
        self._solver_worker.stop()
        self._solver_thread.quit()


class SolverWorker(QObject):

    finished = pyqtSignal()
    step_update = pyqtSignal()

    def __init__(self, controller, rate: int) -> None:
        super().__init__()
        self.running = False

        self.controller: Ui_Controller = controller

        self._rate = rate
        self._last_dq = None

    def moveToThread(self, thread: QThread) -> None:
        super().moveToThread(thread)
        print(self.thread().objectName())

    @pyqtSlot()
    def start(self):
        self.running = True
        self.run()

    def stop(self):
        self.running = False
        print("stopped")

    def run(self):
        rt = rospy.Rate(self._rate)
        self.controller.solver.stack_changed()
        while self.running:
            self.control_step()
            rt.sleep()

        print("stopped_run")

    def control_step(self):
        lb = np.maximum(
            -0.01,
            (
                self.controller.robot_model.mins * 0.95
                - self.controller.robot_state.joint_values
            )
            / self._rate,
        )

        ub = np.minimum(
            0.01,
            (
                self.controller.robot_model.maxs * 0.95
                - self.controller.robot_state.joint_values
            )
            / self._rate,
        )

        self.controller.hierarchy.compute({})

        dq = self.controller.solver.solve(lb, ub, warmstart=self._last_dq)

        self._last_dq = dq
        if dq is not None:
            self.controller.robot_state.actuate(dq)
            # return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

        # self.control_step_callback()\
        return False


def main():
    rospy.init_node("ik")

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("./python/stack_of_tasks/ui/assets/icon.png"))
    uic = Ui_Controller()
    app.aboutToQuit.connect(uic.save_exit)
    app.exec()


if __name__ == "__main__":
    main()
