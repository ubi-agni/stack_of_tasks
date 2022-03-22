#!/usr/bin/env python

import threading
from solver import Solver
from  task import *
import numpy
import rospy
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3

from visualization_msgs.msg import MarkerArray
from tf import transformations as tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_model import RobotModel, Joint

from markers import *

#random.seed(1)

class Controller(object):

    def __init__(self, pose=TransformStamped(header=
                                                    Header(frame_id='panda_link8'),
                                                    child_frame_id='target',
                                                    transform=Transform(
                                                                rotation=Quaternion(*tf.quaternion_about_axis(numpy.pi/4, [0, 0, 1])),
                                                                translation=Vector3(0, 0, 0.105)))
                ):

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform

        self.joint_pub = rospy.Publisher('/target_joint_states', JointState, queue_size=1, latch=True)
        
        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]

        self.target_link = pose.child_frame_id
        self.N = len(self.robot.active_joints) # number of (active) joints
        
        self.joint_weights = numpy.ones(self.N)
        self.cartesian_weights = numpy.ones(6)

        self.mins = numpy.array([j.min for j in self.robot.active_joints])
        self.maxs = numpy.array([j.max for j in self.robot.active_joints])

        self.prismatic = numpy.array([j.jtype == j.prismatic for j in self.robot.active_joints])
        self.reset()

        self.last_dq = None


    def reset(self, randomness = 0):
        def _rJoint(m,M):
            j = (m + M)/2
            dj = (M - m)/2

            if randomness > 0:
                return j + dj*random.uniform(-randomness, randomness)
            return j

        self.joint_msg.position = list(map(lambda x:_rJoint(*x), zip(self.mins, self.maxs)))
        self.update()

    def update(self):
        self.joint_pub.publish(self.joint_msg)
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))


    def actuate(self, q_delta):
        self.joint_msg.position += q_delta.ravel()
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = numpy.clip(self.joint_msg.position[self.prismatic],
                                                             self.mins[self.prismatic], self.maxs[self.prismatic])

        self.update()


    def hierarchic_control(self, **targets):
        T_tar = targets['pose']

        tasks = []

        plane = PlaneTask(1)
        para = ParallelTask([0,0,1], [0,0,1])
        dist = DisstanceTask(6)
        speed = ConstantSpeed(1)
        combine = CombineTasks()

        tasks.append(plane.compute(self.J, T_tar, self.T))
        tasks.append(para.compute(self.J, self.T, T_tar))
        tasks.append(combine.compute(
            speed.compute(self.J,T_tar, self.T, [0,0,1], 0.01),
            dist.compute(self.J, T_tar, self.T, 0.15)
        ))


        lb = np.maximum(-0.1, self.mins - self.joint_msg.position)
        ub = np.minimum(0.1, self.maxs - self.joint_msg.position)

        s = Solver(self.N, lb, ub, 0.15)

        q_delta, tcr = s.solve_sot(tasks, warmstart=self.last_dq)
        self.last_dq = q_delta

        if q_delta is not None:
            self.actuate(q_delta)
            #return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

        return False

class MarkerControl:

    def __init__(self) -> None:
        self.targets = {}

        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.ims = InteractiveMarkerServer('controller') 

        T = tf.translation_matrix([0.8,0,0.5])
        addMarker(self.ims, iPoseMarker(T, markers=[sphere(), plane()], name='pose'), processFeedback(self.setTarget))

        self.ims.applyChanges()

    def setTarget(self, name, goal):
        self.targets[name] = goal


class ControlThread(threading.Thread):
    def __init__(self, func):
        super().__init__(daemon=True)

        self.dorun = False
        self.func = func


    def run(self) -> None:
        self.dorun = True

        while self.dorun:
            x = self.func()
            self.dorun = self.dorun & x
        print("\nControl ended")

    def stop(self):
        self.dorun = False


if __name__ == '__main__':
    rospy.init_node('ik')
    rate = rospy.Rate(50)
    
    c = Controller()
    mc = MarkerControl()

    def loop():
        rval = not c.hierarchic_control(**mc.targets)
        rate.sleep()
        return rval
#
    


    print("Stop with CRTL-D")
    while True:
        input("Start control with [ENTER]")
        t = ControlThread(loop)
        t.start()
        input("Stop control with [ENTER]")
        t.stop()
        t.join()




