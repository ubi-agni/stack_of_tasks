#!/usr/bin/env python3

import numpy as np

import rospy
from tf import transformations as tf
from geometry_msgs.msg import Pose, Quaternion, Vector3

from stack_of_tasks.controller import Controller
from stack_of_tasks.plot.plot_publisher import PlotPublisher
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType
from stack_of_tasks.utils import OffsetTransform


class Main:
    def __init__(self):

        self.controller = {}
        self.targets = {}
        self.dqs = {}
        self.plot = PlotPublisher()

        opt = {"rho": 1.0}
        c1, frame = self.add_controller(HQPSolver, "new", **opt)
        p1 = PositionTask(frame, TaskSoftnessType.linear, 1.0)
        o1 = OrientationTask(frame, TaskSoftnessType.linear, 1.0)
        c1.task_hierarchy.add_task_lower(p1)
        c1.task_hierarchy.add_task_same(o1)

        c2, frame = self.add_controller(HQPSolver, "old", **opt)
        p2 = PositionTask(frame, TaskSoftnessType.quadratic, 1.0)
        o2 = OrientationTask(frame, TaskSoftnessType.quadratic, 1.0)
        c2.task_hierarchy.add_task_lower(p2)
        c2.task_hierarchy.add_task_same(o2)

        c3, frame = self.add_controller(InverseJacobianSolver, "jac", **opt)
        p3 = PositionTask(frame, TaskSoftnessType.quadratic, 1.0)
        o3 = OrientationTask(frame, TaskSoftnessType.quadratic, 1.0)
        c3.task_hierarchy.add_task_lower(p3)
        c3.task_hierarchy.add_task_same(o3)

    def set_target(self, name, data):
        self.targets[name] = data

    def add_controller(self, solverclass, name, **options):
        controller = Controller(solverclass, ns_prefix=f"{name}/", **options)
        self.controller[name] = controller

        self.plot.add_plot(
            f"{name}/dq",
            [f"{name}/dq/{joint.name}" for joint in controller.robot.active_joints],
        )

        controller.control_step_callback.append(
            lambda: self.plot.plot(f"{name}/dq", controller.last_dq)
        )

        controller.reset()

        frame = OffsetTransform(f"{name}_hand_tcp")
        return controller, frame

    def run(self):
        rate = rospy.Rate(50)

        t_1 = tf.rotation_matrix(np.pi, [1, 0, 0])
        t_1[:3, 3] = [0.3, 0, 0.5]

        t_2 = tf.rotation_matrix(np.pi + 0.2, [1, 0, 0])
        t_2[:3, 3] = [0.5, 0, 0.5]

        t = [OffsetTransform("world", t_1), OffsetTransform("world", t_2)]
        i = 0
        self.targets["target"] = t[i]

        while not rospy.is_shutdown():
            for c in self.controller.values():
                c.hierarchic_control(self.targets)

            all_close = True
            for name, ctrl in self.controller.items():
                all_close &= np.allclose(ctrl.last_dq, 0, atol=1e-10)

            if all_close:
                i = (i + 1) % 2
                self.targets["target"] = t[i]
                print("Next target", i)
                rospy.sleep(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")
    m = Main()
    m.run()
