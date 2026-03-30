from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass
class GraspAttempt:
    success: bool
    reason: str
    distance: float | None = None
    relative_speed: float | None = None


class VirtualGripper:
    """Constraint-based virtual gripper for robots without a modeled jaw pair."""

    def __init__(
        self,
        pb,
        robot_id: int,
        end_effector_link_index: int,
        grasp_distance_threshold: float = 0.085,
        max_relative_speed: float = 0.25,
    ) -> None:
        self.pb = pb
        self.robot_id = robot_id
        self.end_effector_link_index = end_effector_link_index
        self.grasp_distance_threshold = grasp_distance_threshold
        self.max_relative_speed = max_relative_speed
        self.constraint_id: int | None = None
        self.attached_body_id: int | None = None
        self.disabled_collision_pairs: list[tuple[int, int]] = []

    def is_holding_object(self) -> bool:
        return self.constraint_id is not None and self.attached_body_id is not None

    def open(self) -> None:
        if self.constraint_id is not None:
            self.pb.removeConstraint(self.constraint_id)
            self.constraint_id = None
            self.attached_body_id = None
        for body_a, body_b in self.disabled_collision_pairs:
            self.pb.setCollisionFilterPair(body_a, body_b, -1, -1, 1)
        self.disabled_collision_pairs.clear()

    def measure_target_alignment(self, target_body_id: int) -> tuple[float, float]:
        contact_points = self.pb.getContactPoints(
            bodyA=self.robot_id,
            bodyB=target_body_id,
            linkIndexA=self.end_effector_link_index,
        )
        if contact_points:
            min_distance = min(point[8] for point in contact_points)
        else:
            close_points = self.pb.getClosestPoints(
                bodyA=self.robot_id,
                bodyB=target_body_id,
                distance=max(0.10, self.grasp_distance_threshold * 4.0),
                linkIndexA=self.end_effector_link_index,
            )
            min_distance = min((point[8] for point in close_points), default=float("inf"))

        end_effector_state = self.pb.getLinkState(
            self.robot_id,
            self.end_effector_link_index,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )
        end_effector_linear_velocity = end_effector_state[6]
        target_linear_velocity, _ = self.pb.getBaseVelocity(target_body_id)
        relative_speed = sum((a - b) ** 2 for a, b in zip(end_effector_linear_velocity, target_linear_velocity)) ** 0.5
        return float(min_distance), float(relative_speed)

    def is_grasp_ready(self, target_body_id: int) -> bool:
        distance, relative_speed = self.measure_target_alignment(target_body_id)
        return distance <= self.grasp_distance_threshold and relative_speed <= self.max_relative_speed

    def close(self, target_body_id: int, disable_collisions_with: Sequence[int] | None = None) -> GraspAttempt:
        if self.is_holding_object():
            return GraspAttempt(True, "object already attached")

        distance, relative_speed = self.measure_target_alignment(target_body_id)
        if distance > self.grasp_distance_threshold:
            return GraspAttempt(False, "target is outside the grasp threshold", distance=distance, relative_speed=relative_speed)
        if relative_speed > self.max_relative_speed:
            return GraspAttempt(False, "target is still moving too fast", distance=distance, relative_speed=relative_speed)

        end_effector_state = self.pb.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        ee_position, ee_orientation = end_effector_state[4], end_effector_state[5]
        target_position, target_orientation = self.pb.getBasePositionAndOrientation(target_body_id)

        inverse_ee_position, inverse_ee_orientation = self.pb.invertTransform(ee_position, ee_orientation)
        relative_position, relative_orientation = self.pb.multiplyTransforms(
            inverse_ee_position,
            inverse_ee_orientation,
            target_position,
            target_orientation,
        )

        self.constraint_id = self.pb.createConstraint(
            self.robot_id,
            self.end_effector_link_index,
            target_body_id,
            -1,
            self.pb.JOINT_FIXED,
            [0.0, 0.0, 0.0],
            relative_position,
            [0.0, 0.0, 0.0],
            parentFrameOrientation=relative_orientation,
            childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
        )
        self.pb.changeConstraint(self.constraint_id, maxForce=400.0)
        self.pb.resetBaseVelocity(target_body_id, linearVelocity=[0.0, 0.0, 0.0], angularVelocity=[0.0, 0.0, 0.0])
        self.pb.changeDynamics(target_body_id, -1, linearDamping=0.12, angularDamping=0.12)
        if disable_collisions_with:
            for other_body_id in disable_collisions_with:
                self.pb.setCollisionFilterPair(target_body_id, other_body_id, -1, -1, 0)
                self.disabled_collision_pairs.append((target_body_id, other_body_id))
        self.attached_body_id = target_body_id
        return GraspAttempt(True, "constraint created", distance=distance, relative_speed=relative_speed)
