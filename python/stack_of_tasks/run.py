#!/usr/bin/env python

from threading import Thread

import rospy
from PyQt5.QtWidgets import QApplication

from tf import transformations as tf

from controller import Controller, MarkerControl
from stack_of_tasks.markers import (OrientationMarker, PositionMarker,
                                    SixDOFMarker)
from stack_of_tasks.task import (ConeTask, JointPos, OrientationTask,
                                 PlaneTask, PositionTask, PreventJointBounds,
                                 Task, TaskHirachy, __available_task_classes__)
from stack_of_tasks.ui.mainwindow import Ui


class Main():
    def __init__(self) -> None:
        self.marker = MarkerControl()
        self.task_hirachy = TaskHirachy()
        self.controller = Controller(rate=50)

        self.ui = Ui()

        self.ui.marker.set_available_marker_classes([PositionMarker, OrientationMarker, SixDOFMarker])
        self.ui.marker.new_marker.connect(self._create_marker)


        self.marker.set_data_callback.append(self.ui.marker.targets.set_target)


    def _insert_task_index(self, task_class, index):
        task = task_class()
        self.task_hirachy.add_task_at(task, index)
        
        #self.ui.hirachy insert task in hirachy



    def _create_marker(self, MClass, name, posittion, rotation, scale):
        pose = tf.translation_matrix(posittion)
        pose = pose.dot(tf.euler_matrix(*rotation))
        m = MClass(name=name, pose=pose, scale=scale)
        self.marker.add_marker(m, "name")


    def setup_h(self):
        # setup tasks
        pos = PositionTask(1)
        pos.argmap["T_t"] = "Position"

        cone = ConeTask((0, 0, 1), (0, 0, 1), 1)
        cone.argmap["T_t"] = "Position"
        cone.argmap["angle"] = "Cone_angle"

        # ori = OrientationTask(3)
        # ori.argmap['T_t'] = 'Position'


        self.controller.task_hirachy.add_task_lower(pos)
        self.controller.task_hirachy.add_task_lower(cone)
        
    
    def start_hirachy_control(self):
        self.control_is_running = True
        def run_func():
            rate = rospy.Rate(50)
            while self.control_is_running:
                self.controller.hierarchic_control(self.marker.targets)
                rate.sleep()

        self._thread = Thread(target=run_func, daemon=True).start()


if __name__ == "__main__":
    rospy.init_node('ik')
    
    import sys
    app = QApplication(sys.argv)
    main = Main()
    app.exec()

