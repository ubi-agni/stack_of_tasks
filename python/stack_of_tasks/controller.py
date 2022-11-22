import numpy as np

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf import transformations as tf

from stack_of_tasks.marker.interactive_marker import IAMarker
from stack_of_tasks.robot_model import JointStatePublisher, RobotModel, RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.utils import Callback

# random.seed(1)


class Controller(object):
    def __init__(
        self,
        solver_class: Solver,
        rate=50,
        robot_model=None,
        ns_prefix="",
        **solver_options,
    ):

        self.rate = rate

        self.joint_callback = Callback()
        self.control_step_callback = Callback()

        if robot_model is None:
            robot_model = RobotModel()

        self.robot_model = robot_model
        self.robot_state = RobotState(robot_model, ns_prefix=ns_prefix)

        self.task_hierarchy = TaskHierarchy()
        self.solver = solver_class(self.robot_model.N, self.task_hierarchy, **solver_options)

        self.last_dq = None

    def check_end(self):
        pass

    def hierarchic_control(self, targets):
        lb = np.maximum(
            -0.01, (self.robot_model.mins * 0.95 - self.robot_state.joint_values) / self.rate
        )
        ub = np.minimum(
            0.01, (self.robot_model.maxs * 0.95 - self.robot_state.joint_values) / self.rate
        )

        self.task_hierarchy.compute(targets)

        dq = self.solver.solve(lb, ub, warmstart=self.last_dq)

        self.last_dq = dq
        if dq is not None:
            self.robot_state.actuate(dq)
            # return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

        self.control_step_callback()
        return False


class MarkerControl:
    def __init__(self) -> None:
        self.ims = InteractiveMarkerServer("controller")
        self.marker = {}
        self.marker_data_callback = Callback()

    def add_marker(self, marker: IAMarker, name):
        self.marker[name] = marker
        marker.data_callbacks.append(self.marker_data_callback)
        marker.init_server(self.ims)

    def delete_marker(self, name):
        self.marker[name].delete()
        # t = self.marker[name].provided_targets()
        # for x in t:
        #    print(x)
        #    del self.targets[x]
        del self.marker[name]
