from __future__ import annotations

from enum import IntEnum

from typing_extensions import Annotated

import pybullet as pb
from pydantic import BeforeValidator
from pydantic.dataclasses import dataclass


def bytes_to_str(value):
    if isinstance(value, bytes):
        return value.decode()
    else:
        return value


BytesStr = Annotated[str, BeforeValidator(bytes_to_str)]


class BulletConnectionMode(IntEnum):
    """Bullet Connection Modes"""

    GUI = pb.GUI
    """Start bullet ui."""
    DIRECT = pb.DIRECT
    """Run bullet headless."""
    # SHARED_MEMORY = pb.SHARED_MEMORY
    # UDP = pb.UDP
    # TCP = pb.TCP


class BulletException(RuntimeError):
    pass


class BulletNotConnectedException(BulletException):
    pass


class Ops(IntEnum):
    ADD = 0
    REMOVE = 1
    APPEND = 2
    MOVE = 3


class PrimitiveType(IntEnum):
    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4


@dataclass
class ContactPoint:
    """A ContactPoint returned by pybullet."""

    contactFlag: int
    bodyUniqueIdA: int
    bodyUniqueIdB: int
    linkIndexA: int
    linkIndexB: int
    positionOnA: list[float]
    positionOnB: list[float]
    contactNormalOnB: list[float]
    contactDistance: float
    normalForce: float
    lateralFriction1: float
    lateralFrictionDir1: list[float]
    lateralFriction2: float
    lateralFrictionDir2: list[float]


@dataclass
class BulletJointInfoReturnVal:
    """A JointInfo returned by pybullet."""

    jointIndex: int
    jointName: BytesStr
    jointType: int
    qIndex: int
    uIndex: int
    flags: int
    jointDamping: float
    jointFriction: float
    jointLowerLimit: float
    jointUpperLimit: float
    jointMaxForce: float
    jointMaxVelocity: float
    linkName: BytesStr
    jointAxis: list[float]
    parentFramePos: list[float]
    parentFrameOrn: list[float]
    parentIndex: int
