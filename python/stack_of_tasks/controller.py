#!/usr/bin/env python3

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
        robot_model=None,
        rate=50,
        transform=tf.quaternion_matrix([0, 0, 0.382, 0.924]).dot(
            tf.translation_matrix([0, 0, 0.105])
        ),
        target_link="panda_joint8",
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
        self.robot_state.joints_changed.append(self.robot_joints_changed)

        self.target_link = target_link
        self.target_offset = transform

        self.T, self.J = self.robot_state.fk(self.target_link)

        self.task_hierarchy = TaskHierarchy()
        self.solver = solver_class(self.robot_model.N, self.task_hierarchy, **solver_options)

        self.last_dq = None

    def robot_joints_changed(self):
        self.T, self.J = self.robot_state.fk(self.target_link)
        self.T = self.T.dot(self.target_offset)

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


if __name__ == "__main__":
    from stack_of_tasks.marker.markers import ConeMarker, SixDOFMarker
    from stack_of_tasks.solver.CVXOPTSolver import CVXOPTSolver
    from stack_of_tasks.solver.OSQPSolver import OSQPSolver
    from stack_of_tasks.solver.SCSSolver import SCSSolver
    from stack_of_tasks.tasks.Eq_Tasks import JointPos, OrientationTask, PositionTask
    from stack_of_tasks.tasks.Ieq_Tasks import ConeTask
    from stack_of_tasks.tasks.Task import TaskSoftnessType

    np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")

    rospy.init_node("ik")
    rate = rospy.Rate(50)

    def set_target(name, data):
        targets[name] = data

    targets = {}

    c = Controller(solver_class=OSQPSolver, rho=0.01)
    p = JointStatePublisher(c.robot_state, "")

    c.control_step_callback.append(lambda: set_target("T", c.T))
    c.control_step_callback.append(lambda: set_target("J", c.J))

    c.control_step_callback()

    mc = MarkerControl()
    mc.marker_data_callback.append(set_target)

    # marker = SixDOFMarker(name="pose", scale=0.1, pose=targets["T"])
    # mc.add_marker(marker, marker.name)

    marker = ConeMarker(name="cone", scale=0.2, pose=targets["T"])
    mc.add_marker(marker, marker.name)

    # setup tasks
    pos = PositionTask(1, TaskSoftnessType.linear)
    pos.set_argument_mapping("current", "T")
    pos.set_argument_mapping("target", f"{marker.name}_pose")

    # ori = OrientationTask(1, TaskSoftnessType.quadratic)
    # ori.set_argument_mapping("current", "T")
    # ori.set_argument_mapping("target", marker.name)

    targets["threshold"] = 1
    targets["target_axis"] = np.array([0, 0, 1])
    targets["robot_axis"] = np.array([0, 0, 1])

    cone = ConeTask(1, TaskSoftnessType.quadratic)
    cone.set_argument_mapping("current", "T")
    cone.set_argument_mapping("target", f"{marker.name}_pose")
    cone.set_argument_mapping("threshold", f"{marker.name}_angle")

    c.task_hierarchy.add_task_lower(pos)
    c.task_hierarchy.add_task_lower(cone)

    # pp = PlotPublisher()
    # pp.add_plot("q", [f"q/{joint.name}" for joint in c.robot.active_joints])
    # pp.add_plot("dq", [f"dq/{joint.name}" for joint in c.robot.active_joints])
    # c.joint_state_callback.append(lambda q: pp.plot("q", q))
    # c.delta_q_callback.append(lambda dq: pp.plot("dq", dq))

    c.solver.stack_changed()

    while not rospy.is_shutdown():
        c.hierarchic_control(targets)
        rate.sleep()
