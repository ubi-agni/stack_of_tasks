#!/usr/bin/env python3

from stack_of_tasks.controller import Controller
from stack_of_tasks.plot.plot_publisher import PlotPublisher
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.Tasks import ConeTask, OrientationTask, PositionTask
import numpy as np
from tf import transformations as tf
import rospy


class Main:
    def __init__(self):

        self.controller = {}
        self.targets = {}
        self.dqs = {}
        self.plot = PlotPublisher()

        opt = {"rho": 1.0}
        self.add_controller(HQPSolver, "new", **opt)
        self.add_controller(HQPSolver, "old", solver="components", **opt)
        self.add_controller(InverseJacobianSolver, "jac", **opt)

    def set_target(self, name, data):
        self.targets[name] = data

    def add_controller(self, solverclass, name, **options):
        controller = Controller(
            solverclass, target_link=f"{name}_joint8", ns_prefix=f"{name}/", **options
        )
        self.controller[name] = controller
        controller.T_callback.append(lambda T: self.set_target(f"T_{name}", T))
        controller.J_callback.append(lambda J: self.set_target(f"J_{name}", J))

        pos = PositionTask(1.0)
        pos.argmap["T_c"] = f"T_{name}"
        pos.argmap["J"] = f"J_{name}"

        ori = OrientationTask(1.0)
        ori.argmap["T_c"] = f"T_{name}"
        ori.argmap["J"] = f"J_{name}"

        controller.task_hierarchy.add_task_lower(pos)
        controller.task_hierarchy.add_task_same(ori)

        self.plot.add_plot(
            f"{name}/dq",
            [f"{name}/dq/{joint.name}" for joint in controller.robot.active_joints],
        )

        controller.delta_q_callback.append(lambda dq: self.plot.plot(f"{name}/dq", dq))
        controller.delta_q_callback.append(lambda dq: self.set_target(f"{name}_dq", dq))

        controller.reset()

    def run(self):
        rate = rospy.Rate(50)

        t_1 = tf.rotation_matrix(np.pi, [1, 0, 0])
        t_1[:3, 3] = [0.3, 0, 0.5]

        t_2 = tf.rotation_matrix(np.pi + 0.2, [1, 0, 0])
        t_2[:3, 3] = [0.5, 0, 0.5]

        t = [t_1, t_2]
        i = 0
        self.targets["T_t"] = t[i]

        while not rospy.is_shutdown():
            for c in self.controller.values():
                c.hierarchic_control(self.targets)

            all_close = True
            for n in self.controller.keys():
                all_close &= np.allclose(
                    self.targets[f"T_{n}"], self.targets["T_t"], atol=1e-4
                )
                all_close &= np.allclose(self.targets[f"{n}_dq"], 0, atol=1e-10)

            if all_close:
                i = (i + 1) % 2
                self.targets["T_t"] = t[i]
                print("Next target", i)
                rospy.sleep(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")
    m = Main()
    m.run()
