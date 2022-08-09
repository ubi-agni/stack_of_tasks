#!/usr/bin/env python3

from stack_of_tasks.controller import Controller
from stack_of_tasks.plot.plot_publisher import PlotPublisher
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.Tasks import ConeTask, OrientationTask, PositionTask
import numpy as np
from tf import transformations as tf
import rospy


class HQPSolverTwo(HQPSolver):

    def solve(self, stack_of_tasks, lower_dq, upper_dq, **options):
        l = np.NINF
        u = np.PINF

        qdot, tcr =  super().solve(stack_of_tasks,l,u, **options)
        scales = np.maximum(qdot / lower_dq, qdot / upper_dq)
        qdot /= np.maximum(1.0, np.max(scales))

        return qdot, tcr


class Main:

    def __init__(self):
        
        self.controller = {}
        self.targets = {}
        self.dqs = {}
        self.plot = PlotPublisher("plot")

        opt = {'rho': 0.1}
        self.add_controller(HQPSolver, opt, "hqp1" )
        self.add_controller(HQPSolverTwo, opt, "hqp2" )
        self.add_controller(InverseJacobianSolver, opt, "inv" )



    def set_target(self, name, data):
            self.targets[name] = data
    

    def add_controller(self, solverclass, solver_options, name):

        controller = Controller(solverclass, publish_joints=False, solver_options=solver_options)
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

        controller.delta_q_callback.append(lambda dq: self.plot.plot(dq, f"{name}/dq"))
        controller.delta_q_callback.append(lambda dq: self.set_target(f"{name}_dq", dq))

        controller.reset()


    def run(self):
        rate = rospy.Rate(50)

        t_1 = tf.rotation_matrix(np.pi, [1,0,0])
        t_1[:3,3] = [.3,0,.5]

        t_2 = tf.rotation_matrix(np.pi+0.2, [1,0,0])
        t_2[:3,3] = [.5,0,.5]
        
        t = [t_1, t_2]
        i = 0
        self.targets["T_t"] = t[i]

        while not rospy.is_shutdown():
            for c in self.controller.values():
                c.hierarchic_control(self.targets)

            all_close = True
            for n in self.controller.keys():
                all_close &= np.allclose(self.targets[f"T_{n}"], self.targets["T_t"], atol=1e-4)
                all_close &= np.allclose(self.targets[f"{n}_dq"],0, atol=1e-10)

            if all_close:
                i = (i+1) % 2
                self.targets["T_t"] = t[i]
                print("Next target", i)
                rospy.sleep(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")
    m = Main()
    m.run()