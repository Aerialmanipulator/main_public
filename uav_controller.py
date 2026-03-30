from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class UAVState:
    position: np.ndarray
    orientation_euler: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


class UAVController:
    """PD hover controller for the equivalent aerial base model."""

    def __init__(
        self,
        pb,
        robot_id: int,
        time_step: float = 1.0 / 240.0,
        kp_xy: float = 18.0,
        kd_xy: float = 10.0,
        kp_z: float = 24.0,
        kd_z: float = 12.0,
        kp_attitude: float = 4.5,
        kd_attitude: float = 1.0,
        kp_yaw: float = 2.5,
        kd_yaw: float = 0.8,
    ) -> None:
        self.pb = pb
        self.robot_id = robot_id
        self.time_step = time_step
        self.kp_xy = kp_xy
        self.kd_xy = kd_xy
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_attitude = kp_attitude
        self.kd_attitude = kd_attitude
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw
        self.total_mass = self._compute_total_mass()

    def _compute_total_mass(self) -> float:
        total_mass = 0.0
        for link_index in range(-1, self.pb.getNumJoints(self.robot_id)):
            total_mass += self.pb.getDynamicsInfo(self.robot_id, link_index)[0]
        return total_mass

    def get_state(self) -> UAVState:
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot_id)
        linear_velocity, angular_velocity = self.pb.getBaseVelocity(self.robot_id)
        euler = self.pb.getEulerFromQuaternion(orientation)
        return UAVState(
            position=np.asarray(position, dtype=float),
            orientation_euler=np.asarray(euler, dtype=float),
            linear_velocity=np.asarray(linear_velocity, dtype=float),
            angular_velocity=np.asarray(angular_velocity, dtype=float),
        )

    def step(self, target_position: Sequence[float], target_yaw: float = 0.0) -> None:
        state = self.get_state()
        target = np.asarray(target_position, dtype=float)

        position_error = target - state.position
        velocity_error = -state.linear_velocity

        desired_linear_velocity = np.array(
            [
                self.kp_xy * position_error[0] + self.kd_xy * velocity_error[0],
                self.kp_xy * position_error[1] + self.kd_xy * velocity_error[1],
                self.kp_z * position_error[2] + self.kd_z * velocity_error[2],
            ],
            dtype=float,
        )
        desired_linear_velocity = np.clip(desired_linear_velocity, [-1.0, -1.0, -0.8], [1.0, 1.0, 1.0])

        _, _, yaw = state.orientation_euler
        yaw_error = self._wrap_to_pi(target_yaw - yaw)
        desired_yaw_rate = float(np.clip(self.kp_yaw * yaw_error - self.kd_yaw * state.angular_velocity[2], -1.2, 1.2))

        # Equivalent-model hover:
        # 1) keep the UAV level by clamping roll/pitch to zero,
        # 2) integrate a translational velocity command from a PD law,
        # 3) explicitly reset the base pose so arm reaction forces do not destabilize the abstracted flying platform.
        next_position = state.position + desired_linear_velocity * self.time_step
        self.pb.resetBasePositionAndOrientation(
            self.robot_id,
            next_position.tolist(),
            self.pb.getQuaternionFromEuler([0.0, 0.0, target_yaw]),
        )
        self.pb.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, desired_yaw_rate],
        )

    def at_target(
        self,
        target_position: Sequence[float],
        position_tolerance: float = 0.05,
        velocity_tolerance: float = 0.12,
    ) -> bool:
        state = self.get_state()
        target = np.asarray(target_position, dtype=float)
        return (
            np.linalg.norm(target - state.position) <= position_tolerance
            and np.linalg.norm(state.linear_velocity) <= velocity_tolerance
        )

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
