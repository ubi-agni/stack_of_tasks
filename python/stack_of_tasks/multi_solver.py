#!/usr/bin/env python3

import numpy as np
import rospy
from tf import transformations as tf

from stack_of_tasks.controller import Controller
from stack_of_tasks.plot.plot_publisher import PlotPublisher
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType


class Main:
    def __init__(self):

        self.controller = {}
        self.targets = {}
        self.dqs = {}
        self.plot = PlotPublisher()

        opt = {"rho": 1.0}
        c1 = self.add_controller(HQPSolver, "new", **opt)

        p1 = PositionTask(1, TaskSoftnessType.linear)
        p1.argmap["current"] = "T_new"
        p1.argmap["J"] = "J_new"

        o1 = OrientationTask(1, TaskSoftnessType.linear)
        o1.argmap["current"] = "T_new"
        o1.argmap["J"] = "J_new"

        c1.task_hierarchy.add_task_lower(p1)
        c1.task_hierarchy.add_task_same(o1)

        c2 = self.add_controller(HQPSolver, "old", **opt)
        p2 = PositionTask(1, TaskSoftnessType.quadratic)
        p2.argmap["current"] = "T_old"
        p2.argmap["J"] = "J_old"

        o2 = OrientationTask(1, TaskSoftnessType.quadratic)
        o2.argmap["current"] = "T_old"
        o2.argmap["J"] = "J_old"

        c2.task_hierarchy.add_task_lower(p2)
        c2.task_hierarchy.add_task_same(o2)

        c3 = self.add_controller(InverseJacobianSolver, "jac", **opt)
        p3 = PositionTask(1, TaskSoftnessType.quadratic)
        p3.argmap["current"] = "T_jac"
        p3.argmap["J"] = "J_jac"

        o3 = OrientationTask(1, TaskSoftnessType.quadratic)
        o3.argmap["current"] = "T_jac"
        o3.argmap["J"] = "J_jac"

        c3.task_hierarchy.add_task_lower(p3)
        c3.task_hierarchy.add_task_same(o3)

    def set_target(self, name, data):
        self.targets[name] = data

    def add_controller(self, solverclass, name, **options):
        controller = Controller(
            solverclass, target_link=f"{name}_joint8", ns_prefix=f"{name}/", **options
        )
        self.controller[name] = controller

        controller.joint_callback.append(lambda: self.set_target(f"T_{name}", controller.T))
        controller.joint_callback.append(lambda: self.set_target(f"J_{name}", controller.J))

        self.plot.add_plot(
            f"{name}/dq",
            [f"{name}/dq/{joint.name}" for joint in controller.robot.active_joints],
        )

        controller.control_step_callback.append(
            lambda: self.plot.plot(f"{name}/dq", controller.last_dq)
        )
        controller.control_step_callback.append(
            lambda: self.set_target(f"{name}_dq", controller.last_dq)
        )

        controller.reset()
        return controller

    def run(self):
        rate = rospy.Rate(50)

        t_1 = tf.rotation_matrix(np.pi, [1, 0, 0])
        t_1[:3, 3] = [0.3, 0, 0.5]

        t_2 = tf.rotation_matrix(np.pi + 0.2, [1, 0, 0])
        t_2[:3, 3] = [0.5, 0, 0.5]

        t = [t_1, t_2]
        i = 0
        self.targets["target"] = t[i]

        while not rospy.is_shutdown():
            for c in self.controller.values():
                c.hierarchic_control(self.targets)

            all_close = True
            for n in self.controller.keys():
                all_close &= np.allclose(
                    self.targets[f"T_{n}"], self.targets["target"], atol=1e-4
                )
                all_close &= np.allclose(self.targets[f"{n}_dq"], 0, atol=1e-10)

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
