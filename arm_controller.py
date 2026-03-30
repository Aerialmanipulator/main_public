from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import numpy as np


class ArmController:
    """DLS IK controller for the arm mounted under the UAV body."""

    def __init__(
        self,
        pb,
        robot_id: int,
        prepared_urdf_path: str | Path,
        arm_joint_names: Sequence[str],
        end_effector_link_name: str = "arm_link_5",
        joint_step_limit: float = 0.035,
        max_motor_force: float = 35.0,
    ) -> None:
        self.pb = pb
        self.robot_id = robot_id
        self.prepared_urdf_path = Path(prepared_urdf_path)
        self.arm_joint_names = list(arm_joint_names)
        self.end_effector_link_name = end_effector_link_name
        self.joint_step_limit = joint_step_limit
        self.max_motor_force = max_motor_force

        self.arm_joint_indices = self._find_joint_indices(robot_id, self.arm_joint_names)
        self.end_effector_link_index = self._find_link_index(robot_id, end_effector_link_name)
        self.lower_limits, self.upper_limits = self._read_joint_limits(robot_id, self.arm_joint_indices)
        default_home_positions = np.array([0.0, 0.0, 0.0, -math.pi, 0.0], dtype=float)
        self.home_positions = np.clip(
            default_home_positions[: len(self.arm_joint_indices)],
            self.lower_limits,
            self.upper_limits,
        )

        # A second hidden client is used as a fixed-base kinematic proxy.
        # This keeps Jacobian-based IK numerically stable while the real robot remains free-flying.
        self.kinematics_client = self.pb.connect(self.pb.DIRECT)
        self.kinematics_robot_id = self.pb.loadURDF(
            self.prepared_urdf_path.as_posix(),
            useFixedBase=True,
            flags=self.pb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.kinematics_client,
        )
        self.kinematics_joint_indices = self._find_joint_indices(
            self.kinematics_robot_id,
            self.arm_joint_names,
            physics_client_id=self.kinematics_client,
        )
        self.kinematics_end_effector_index = self._find_link_index(
            self.kinematics_robot_id,
            self.end_effector_link_name,
            physics_client_id=self.kinematics_client,
        )

    def _find_joint_indices(
        self,
        robot_id: int,
        joint_names: Sequence[str],
        physics_client_id: int | None = None,
    ) -> list[int]:
        mapping = {}
        num_joints = self._get_num_joints(robot_id, physics_client_id)
        for joint_index in range(num_joints):
            name = self._get_joint_info(robot_id, joint_index, physics_client_id)[1].decode("utf-8")
            mapping[name] = joint_index
        missing = [name for name in joint_names if name not in mapping]
        if missing:
            raise KeyError(f"Joint(s) not found in robot: {missing}")
        return [mapping[name] for name in joint_names]

    def _find_link_index(self, robot_id: int, link_name: str, physics_client_id: int | None = None) -> int:
        num_joints = self._get_num_joints(robot_id, physics_client_id)
        for joint_index in range(num_joints):
            name = self._get_joint_info(robot_id, joint_index, physics_client_id)[12].decode("utf-8")
            if name == link_name:
                return joint_index
        raise KeyError(f"Link not found in robot: {link_name}")

    def _get_num_joints(self, robot_id: int, physics_client_id: int | None = None) -> int:
        if physics_client_id is None:
            return self.pb.getNumJoints(robot_id)
        return self.pb.getNumJoints(robot_id, physicsClientId=physics_client_id)

    def _get_joint_info(self, robot_id: int, joint_index: int, physics_client_id: int | None = None):
        if physics_client_id is None:
            return self.pb.getJointInfo(robot_id, joint_index)
        return self.pb.getJointInfo(robot_id, joint_index, physicsClientId=physics_client_id)

    def _read_joint_limits(self, robot_id: int, joint_indices: Sequence[int]) -> tuple[np.ndarray, np.ndarray]:
        lower_limits = []
        upper_limits = []
        for joint_index in joint_indices:
            info = self.pb.getJointInfo(robot_id, joint_index)
            lower = info[8]
            upper = info[9]
            if lower >= upper:
                lower, upper = -math.pi, math.pi
            lower_limits.append(lower)
            upper_limits.append(upper)
        return np.asarray(lower_limits, dtype=float), np.asarray(upper_limits, dtype=float)

    def get_joint_positions(self) -> np.ndarray:
        return np.asarray(
            [self.pb.getJointState(self.robot_id, joint_index)[0] for joint_index in self.arm_joint_indices],
            dtype=float,
        )

    def get_end_effector_pose(self) -> tuple[np.ndarray, np.ndarray]:
        state = self.pb.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        return np.asarray(state[4], dtype=float), np.asarray(state[5], dtype=float)

    def predict_end_effector_pose(
        self,
        base_position: Sequence[float],
        base_orientation: Sequence[float] | None = None,
        joint_positions: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0, 1.0]
        if joint_positions is None:
            joint_positions = self.home_positions

        self.pb.resetBasePositionAndOrientation(
            self.kinematics_robot_id,
            base_position,
            base_orientation,
            physicsClientId=self.kinematics_client,
        )
        for joint_index, joint_value in zip(self.kinematics_joint_indices, joint_positions):
            self.pb.resetJointState(
                self.kinematics_robot_id,
                joint_index,
                float(joint_value),
                physicsClientId=self.kinematics_client,
            )

        state = self.pb.getLinkState(
            self.kinematics_robot_id,
            self.kinematics_end_effector_index,
            computeForwardKinematics=True,
            physicsClientId=self.kinematics_client,
        )
        return np.asarray(state[4], dtype=float), np.asarray(state[5], dtype=float)

    def set_joint_targets(self, joint_positions: Sequence[float], max_force: float | None = None) -> None:
        current_positions = self.get_joint_positions()
        clipped_targets = np.clip(
            np.asarray(joint_positions, dtype=float),
            current_positions - self.joint_step_limit,
            current_positions + self.joint_step_limit,
        )

        applied_force = self.max_motor_force if max_force is None else max_force
        for joint_index, target_position in zip(self.arm_joint_indices, clipped_targets):
            self.pb.setJointMotorControl2(
                self.robot_id,
                joint_index,
                self.pb.POSITION_CONTROL,
                targetPosition=float(target_position),
                force=applied_force,
                positionGain=0.18,
                velocityGain=0.10,
            )

    def command_home(self) -> None:
        self.set_joint_targets(self.home_positions)

    def command_end_effector(
        self,
        target_position: Sequence[float],
        target_orientation: Sequence[float] | None = None,
    ) -> float:
        solution, residual = self.solve_dls(target_position, target_orientation)
        self.set_joint_targets(solution)
        return residual

    def at_pose(self, target_position: Sequence[float], tolerance: float = 0.035) -> bool:
        current_position, _ = self.get_end_effector_pose()
        return np.linalg.norm(np.asarray(target_position, dtype=float) - current_position) <= tolerance

    def solve_dls(
        self,
        target_position: Sequence[float],
        target_orientation: Sequence[float] | None = None,
        damping: float = 0.08,
        step_size: float = 0.7,
        max_iterations: int = 80,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.08,
    ) -> tuple[np.ndarray, float]:
        self._sync_kinematics_model()

        target_position_array = np.asarray(target_position, dtype=float)
        q = self.get_joint_positions().copy()
        residual = float("inf")

        for _ in range(max_iterations):
            for joint_index, joint_value in zip(self.kinematics_joint_indices, q):
                self.pb.resetJointState(
                    self.kinematics_robot_id,
                    joint_index,
                    float(joint_value),
                    physicsClientId=self.kinematics_client,
                )

            state = self.pb.getLinkState(
                self.kinematics_robot_id,
                self.kinematics_end_effector_index,
                computeForwardKinematics=True,
                physicsClientId=self.kinematics_client,
            )
            current_position = np.asarray(state[4], dtype=float)
            current_orientation = np.asarray(state[5], dtype=float)

            position_error = target_position_array - current_position
            residual = float(np.linalg.norm(position_error))

            task_error = position_error
            orientation_error_norm = 0.0

            jacobian_linear, jacobian_angular = self._compute_arm_jacobian(q, current_position, current_orientation)
            task_jacobian = jacobian_linear

            if target_orientation is not None:
                orientation_error = self._orientation_error(current_orientation, target_orientation)
                orientation_error_norm = float(np.linalg.norm(orientation_error))
                task_error = np.concatenate([position_error, orientation_error], axis=0)
                task_jacobian = np.vstack([jacobian_linear, jacobian_angular])

            if residual <= position_tolerance and orientation_error_norm <= orientation_tolerance:
                break

            identity = np.eye(task_jacobian.shape[0], dtype=float)
            dq = task_jacobian.T @ np.linalg.solve(
                task_jacobian @ task_jacobian.T + (damping**2) * identity,
                task_error,
            )
            q = np.clip(q + step_size * dq, self.lower_limits, self.upper_limits)

        return q, residual

    def _sync_kinematics_model(self) -> None:
        base_position, base_orientation = self.pb.getBasePositionAndOrientation(self.robot_id)
        self.pb.resetBasePositionAndOrientation(
            self.kinematics_robot_id,
            base_position,
            base_orientation,
            physicsClientId=self.kinematics_client,
        )
        for real_joint_index, kinematic_joint_index in zip(self.arm_joint_indices, self.kinematics_joint_indices):
            joint_value = self.pb.getJointState(self.robot_id, real_joint_index)[0]
            self.pb.resetJointState(
                self.kinematics_robot_id,
                kinematic_joint_index,
                float(joint_value),
                physicsClientId=self.kinematics_client,
            )

    def _compute_arm_jacobian(
        self,
        joint_positions: np.ndarray,
        current_position: np.ndarray,
        current_orientation: np.ndarray,
        epsilon: float = 1e-4,
    ) -> tuple[np.ndarray, np.ndarray]:
        jacobian_linear = np.zeros((3, len(self.kinematics_joint_indices)), dtype=float)
        jacobian_angular = np.zeros((3, len(self.kinematics_joint_indices)), dtype=float)

        for column, joint_index in enumerate(self.kinematics_joint_indices):
            perturbed_joint_positions = joint_positions.copy()
            perturbed_joint_positions[column] += epsilon
            for perturbed_index, perturbed_value in zip(self.kinematics_joint_indices, perturbed_joint_positions):
                self.pb.resetJointState(
                    self.kinematics_robot_id,
                    perturbed_index,
                    float(perturbed_value),
                    physicsClientId=self.kinematics_client,
                )

            perturbed_state = self.pb.getLinkState(
                self.kinematics_robot_id,
                self.kinematics_end_effector_index,
                computeForwardKinematics=True,
                physicsClientId=self.kinematics_client,
            )
            perturbed_position = np.asarray(perturbed_state[4], dtype=float)
            perturbed_orientation = np.asarray(perturbed_state[5], dtype=float)

            jacobian_linear[:, column] = (perturbed_position - current_position) / epsilon
            jacobian_angular[:, column] = (
                self._orientation_error(current_orientation, perturbed_orientation) / epsilon
            )

        for joint_index, joint_value in zip(self.kinematics_joint_indices, joint_positions):
            self.pb.resetJointState(
                self.kinematics_robot_id,
                joint_index,
                float(joint_value),
                physicsClientId=self.kinematics_client,
            )

        return jacobian_linear, jacobian_angular

    def _orientation_error(self, current_quaternion: Sequence[float], target_quaternion: Sequence[float]) -> np.ndarray:
        current_inverse = self.pb.invertTransform([0.0, 0.0, 0.0], current_quaternion)[1]
        delta_quaternion = self.pb.multiplyTransforms(
            [0.0, 0.0, 0.0],
            target_quaternion,
            [0.0, 0.0, 0.0],
            current_inverse,
        )[1]
        axis, angle = self.pb.getAxisAngleFromQuaternion(delta_quaternion)
        return np.asarray(axis, dtype=float) * float(angle)

    def shutdown(self) -> None:
        if self.kinematics_client >= 0:
            self.pb.disconnect(self.kinematics_client)
