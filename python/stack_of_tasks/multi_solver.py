#!/usr/bin/env python3

import numpy as np

import rospy
from tf import transformations as tf
from geometry_msgs.msg import Pose, Quaternion, Vector3

from stack_of_tasks.controller import Controller
from stack_of_tasks.plot.plot_publisher import PlotPublisher
from stack_of_tasks.ref_frame.frames import JointFrame, World
from stack_of_tasks.ref_frame.offset import OffsetRefFrame
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

        self.target_frame = OffsetRefFrame(World()).transform(
            tf.quaternion_matrix([1, 0, 0, 0])
        )

        # controller 1

        c1, frame = self.add_controller(HQPSolver, "new", **opt)

        p1 = PositionTask(frame, self.target_frame, TaskSoftnessType.linear, weight=1.0)
        o1 = OrientationTask(frame, self.target_frame, TaskSoftnessType.linear, weight=1.0)

        c1.task_hierarchy.add_task_lower(p1)
        c1.task_hierarchy.add_task_same(o1)

        # controller 2
        c2, frame = self.add_controller(HQPSolver, "old", **opt)
        p2 = PositionTask(frame, self.target_frame, TaskSoftnessType.quadratic, weight=1.0)
        o2 = OrientationTask(frame, self.target_frame, TaskSoftnessType.quadratic, weight=1.0)
        c2.task_hierarchy.add_task_lower(p2)
        c2.task_hierarchy.add_task_same(o2)

        # controller 3
        c3, frame = self.add_controller(InverseJacobianSolver, "jac", **opt)
        p3 = PositionTask(frame, self.target_frame, TaskSoftnessType.quadratic, weight=1.0)
        o3 = OrientationTask(frame, self.target_frame, TaskSoftnessType.quadratic, weight=1.0)
        c3.task_hierarchy.add_task_lower(p3)
        c3.task_hierarchy.add_task_same(o3)

        self.plot.add_plot("target", ["target/x", "target/y", "target/z"])

    def add_controller(self, solverclass, name, **options):
        controller = Controller(solverclass, ns_prefix=f"{name}/", **options)
        self.controller[name] = controller

        self.plot.add_plot(
            f"{name}/dq",
            [f"{name}/dq/{joint.name}" for joint in controller.robot.active_joints],
        )

        self.plot.add_plot(f"pos{name}", [f"pos{name}/x", f"pos{name}/y", f"pos{name}/z"])
        controller.control_step_callback.append(
            lambda: self.plot.plot(f"{name}/dq", controller.last_dq)
        )
        frame = JointFrame(controller=controller, joint=f"{name}_hand_tcp")

        controller.control_step_callback.append(
            lambda: self.plot.plot(name=f"pos{name}", values=frame.T[:3, 3])
        )

        controller.reset()

        return controller, frame

    def run(self):
        rate = rospy.Rate(50)

        self.target_frame.translate(x=0.3, z=-0.5)
        self.plot.plot(name="target", values=self.target_frame.T[:3, 3])
        i = 1
        while not rospy.is_shutdown():
            for c in self.controller.values():
                c.hierarchic_control(self.targets)

            all_close = True
            for name, ctrl in self.controller.items():
                all_close &= np.allclose(ctrl.last_dq, 0, atol=1e-10)

            if all_close:
                self.target_frame.translate(x=(i * 0.2))
                self.plot.plot(name="target", values=self.target_frame.T[:3, 3])
                i -= i * (2)
                rospy.sleep(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")
    m = Main()
    m.run()
