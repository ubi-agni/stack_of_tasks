from __future__ import annotations

from itertools import chain

import numpy as np
import pybullet as pb
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import (
    GetPlanningScene,
    GetPlanningSceneRequest,
    GetPlanningSceneResponse,
)
from shape_msgs.msg import SolidPrimitive

import rospy

from .robot import BulletRobot
from .types import BulletConnectionMode, ContactPoint


class MoveitSyncedBulletScene:
    """This class handles synchronization between the MoveIt planning scene and pybullet. Futhermore it provides convenience functions to load a robot, force-update the scene and perform collision detection."""

    def __init__(self, moveit_scene_topic: str, bullet_mode: BulletConnectionMode) -> None:

        self._bullet_client_id: int = pb.connect(method=bullet_mode)
        self._objs = {}

        try:
            isConnected, connectionMethod = pb.getConnectionInfo(self._bullet_client_id)
        except pb.error as e:
            print(e)

        self._is_connected = True

        rospy.wait_for_service("/get_planning_scene")
        self._ps_service = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)

        self._ps_subscriber = rospy.Subscriber(
            f"/{moveit_scene_topic}", PlanningScene, self._ps_callback
        )

        self._ps_request = GetPlanningSceneRequest()
        self._ps_request.components.components = sum(
            [
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
                PlanningSceneComponents.WORLD_OBJECT_NAMES,
            ]
        )
        self.update_planning_scene()

    def __del__(self):
        if self._is_connected:
            pb.disconnect(self._bullet_client_id)

    def update_planning_scene(self):
        result: GetPlanningSceneResponse = self._ps_service(self._ps_request)
        self._ps_callback(result.scene)

    def _ps_callback(self, scene: PlanningScene):
        c_objs: list[CollisionObject] = (
            scene.world.collision_objects if scene.world.collision_objects is not None else []
        )

        for obj in chain(*self._objs.values()):
            pb.removeBody(obj)

        for c_obj in c_objs:
            self._create_object(c_obj)

    def _create_object(self, obj: CollisionObject):
        scene_id = obj.id
        pose = obj.pose

        for shape in obj.primitives:
            cid = self._create_collision_shape_data(shape)

            mid = pb.createMultiBody(
                baseCollisionShapeIndex=cid,
                basePosition=[pose.position.x, pose.position.y, pose.position.z],
                baseOrientation=[
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            )

            self._objs.setdefault(scene_id, []).append(mid)

    def _create_collision_shape_data(self, shape: SolidPrimitive):

        if shape.type == shape.BOX:
            return pb.createCollisionShape(
                pb.GEOM_BOX, halfExtents=[xi / 2 for xi in shape.dimensions]
            )

        elif shape.type == shape.SPHERE:
            return pb.createCollisionShape(
                pb.GEOM_SPHERE, radius=shape.dimensions[SolidPrimitive.SPHERE_RADIUS]
            )

        elif shape.type == shape.CYLINDER:
            return pb.createCollisionShape(
                pb.GEOM_CYLINDER,
                radius=shape.dimensions[SolidPrimitive.CYLINDER_RADIUS],
                height=shape.dimensions[SolidPrimitive.CYLINDER_HEIGHT],
            )

        else:
            print(f"{shape.type} is not supported.")

    def load_robot(self, urdf_path, robot_state):
        robot_id = pb.loadURDF(
            urdf_path,
            physicsClientId=self._bullet_client_id,
        )

        return BulletRobot(robot_id, robot_state)

    def collision(self, robot: BulletRobot, max_dist=0.1):
        pb.performCollisionDetection()
        pts: list[ContactPoint] = []

        for obj_id in chain(*self._objs.values()):
            pts.extend(
                map(
                    lambda a: ContactPoint(*a),
                    pb.getClosestPoints(robot.bullet_id, obj_id, max_dist),
                )
            )
        return pts
