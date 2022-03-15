#!/usr/bin/env python

from math import comb
import signal
import threading
from solver import Solver
from  task import *
import numpy
import rospy
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform, Pose, Quaternion, Vector3, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory

from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations as tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_model import RobotModel, Joint
from markers import addMarker, cone, iConeMarker, iPlaneMarker, iPositionMarker, plane, processFeedback, iPoseMarker, frame, sphere



class Controller(object):

    def __init__(self, pose=TransformStamped(header=Header(frame_id='panda_link8'), child_frame_id='target',
                                             transform=Transform(rotation=Quaternion(*tf.quaternion_about_axis(numpy.pi/4, [0, 0, 1])),
                                                                 translation=Vector3(0, 0, 0.105)))):

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform
        self.joint_pub = rospy.Publisher('/target_joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]

        self.target_link = pose.child_frame_id

        self.reset()
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))
        self.N = self.J.shape[1]  # number of (active) joints

        self.preferred_joints = self.joint_msg.position.copy()
        self.joint_weights = numpy.ones(self.N)
        self.cartesian_weights = numpy.ones(6)

        self.mins = numpy.array([j.min for j in self.robot.active_joints])
        self.maxs = numpy.array([j.max for j in self.robot.active_joints])
        
        self.prismatic = numpy.array([j.jtype == j.prismatic for j in self.robot.active_joints])

        self.last_dq = None


    def reset(self):
        random.seed(1)
        self.joint_msg.position = numpy.asarray([(j.min+j.max)/2 + 0.1*(j.max-j.min)*random.uniform(0, 1) for j in self.robot.active_joints])
        self.joint_pub.publish(self.joint_msg)

    def actuate(self, q_delta):
        self.joint_msg.position += q_delta.ravel()
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = numpy.clip(self.joint_msg.position[self.prismatic],
                                                             self.mins[self.prismatic], self.maxs[self.prismatic])
        self.joint_pub.publish(self.joint_msg)
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))


    def hierarchic_control(self, **targets):
        T_tar = targets['pose']

        tasks = []

        pos = PositionTask(1)
        ori = OrientationTask()
        combine = CombineTasks()

        posDesc = pos.compute(self.J, T_tar, self.T)
        oriDesc = ori.compute(self.J, self.T, T_tar)
        
        # tasks.append(combine.compute([posDesc, oriDesc]))   # solve orientation and position with same hirachy

        tasks.append(posDesc)   # solve position with higher hirachy than orientation
        tasks.append(oriDesc)

        lb = np.maximum(-0.05, self.mins - self.joint_msg.position)
        ub = np.minimum(0.05, self.maxs - self.joint_msg.position)


        s = Solver(self.N, lb, ub)

        q_delta = s.solve_sot(tasks, warmstart=self.last_dq)
        self.last_dq = q_delta

        if q_delta is not None:
            self.actuate(q_delta)

        return False

class MarkerControl:

    def __init__(self) -> None:
        self.targets = {}

        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.ims = InteractiveMarkerServer('controller') 

        T = tf.translation_matrix([0.8,0,0.5])
        addMarker(self.ims, iPoseMarker(T, markers=[sphere()], name='pose'), processFeedback(self.setTarget))

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
            self.func()

    def stop(self):
        self.dorun = False





if __name__ == '__main__':
    rospy.init_node('ik')
    rate = rospy.Rate(50)
    
    c = Controller()
    c.reset()
    rate.sleep()
    c.reset()
    c.actuate(np.zeros(c.N))
    mc = MarkerControl()

    
    def loop():
        not c.hierarchic_control(**mc.targets)
        rate.sleep()

    while True:
        input("Start control with [ENTER]")

        t = ControlThread(loop)
        t.start()
        input("Stop control with [ENTER]")
        t.stop()
        t.join()
        signal.signal(signal.SIGINT, signal.SIG_DFL)



