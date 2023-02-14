#!/usr/bin/env python3

import sys
import threading

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication

import rospy

from stack_of_tasks.controller import MarkerControl
from stack_of_tasks.marker.markers import SixDOFMarker
from stack_of_tasks.ref_frame.frames import JointFrame
from stack_of_tasks.ref_frame.frames.World import Offset, World
from stack_of_tasks.robot_model import JointStatePublisher, RobotModel, RobotState
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.mainwindow import Ui
from stack_of_tasks.ui.models import RefFramesModel


class Ui_Controller(QObject):
    def __init__(self) -> None:
        super().__init__()

        self.mc = MarkerControl()

        self.hierarchy = TaskHierarchy()
        self.robot_model = RobotModel()
        self.robot_state = RobotState(self.robot_model)
        self.jsp = JointStatePublisher(self.robot_state)

        self.ref_model = RefFramesModel(self.robot_state)
        self.ref_model.add_ref(World(), "ROOT")

        self.solver = OSQPSolver(self.robot_model.N, self.hierarchy, rho=0.01)
        self._solver_worker = None

        self.ui = Ui(self.ref_model, self.hierarchy)
        self.ui.run_Button.clicked.connect(self.run_triggered)

        self.ui.show()

    def init_controller(self, eef: str):
        from stack_of_tasks.tasks import PositionTask
        from stack_of_tasks.tasks.Task import TaskSoftnessType

        target = Offset(World()).rotate(x=0, y=-0.383, z=0, w=0.924).translate(x=0.5, z=0.5)
        eef = JointFrame(self.robot_state, eef)

        m = SixDOFMarker(frame=target, scale=0.25)
        self.mc.add_marker(m, "Target")
        self.ref_model.add_ref(target, "Target")

        task = PositionTask(target, eef, softness=TaskSoftnessType.linear)
        self.hierarchy.append_task(task)

        # Load all refs from existing tasks
        # TODO: Shouldn't hierarchy.append_task() trigger updates of the model?
        for tasks in self.hierarchy:
            self.ui.update_refs_from_tasks(tasks)

    def run_triggered(self):
        if self._solver_worker is not None and self._solver_worker.running:
            self._solver_worker.stop()
        else:
            self._solver_worker = SolverWorker(self, 50)
            self._solver_worker.start()

    @pyqtSlot()
    def save_exit(self):
        if self._solver_worker is not None:
            self._solver_worker.stop()
            self._solver_worker.join(1.0)  # wait for thread to actually finish


class SolverWorker(threading.Thread):
    def __init__(self, controller: Ui_Controller, rate: int) -> None:
        super().__init__()
        self.name = "SolverWorker"
        self.running = False
        self.controller = controller
        self._rate = rate
        self._last_dq = None
        self._joint_values = controller.robot_state.joint_values
        self._mins = 0.95 * self.controller.robot_model.mins
        self._maxs = 0.95 * self.controller.robot_model.maxs

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        rt = rospy.Rate(self._rate)
        self.controller.solver.stack_changed()
        while self.running:
            self.control_step()
            rt.sleep()

        print("stopped run")

    def control_step(self):
        lb = np.maximum(-0.01, (self._mins - self._joint_values) / self._rate)
        ub = np.minimum(+0.01, (self._maxs - self._joint_values) / self._rate)

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
    uic.init_controller("panda_hand_tcp")
    app.aboutToQuit.connect(uic.save_exit)
    app.exec()


if __name__ == "__main__":
    main()
