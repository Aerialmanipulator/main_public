from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from enum import Enum, auto
from pathlib import Path


def ensure_project_venv() -> None:
    project_root = Path(__file__).resolve().parent
    venv_python = project_root / ".venv" / "Scripts" / "python.exe"
    already_reexecuted = os.environ.get("SRP_PROJECT_VENV_ACTIVE") == "1"

    if already_reexecuted or not venv_python.exists():
        return

    if Path(sys.executable).resolve() == venv_python.resolve():
        return

    env = os.environ.copy()
    env["SRP_PROJECT_VENV_ACTIVE"] = "1"
    print(f"Switching to project virtual environment: {venv_python}", flush=True)
    completed = subprocess.run(
        [str(venv_python), str(Path(__file__).resolve()), *sys.argv[1:]],
        env=env,
        check=False,
    )
    raise SystemExit(completed.returncode)


ensure_project_venv()

try:
    import numpy as np
    import pybullet as p
    import pybullet_data
except ImportError as exc:
    raise SystemExit(
        "Missing required dependencies. Install them with `python -m pip install -r requirements.txt`."
    ) from exc

from arm_controller import ArmController
from grasping import VirtualGripper
from simulation_utils import (
    ARM_JOINT_NAMES,
    configure_loaded_robot,
    create_demo_scene,
    describe_joint_layout,
    find_robot_urdf,
    prepare_urdf_for_pybullet,
)
from uav_controller import UAVController


class DemoState(Enum):
    TAKEOFF = auto()
    TRANSIT = auto()
    ALIGN = auto()
    DESCEND = auto()
    APPROACH = auto()
    STABILIZE = auto()
    GRASP = auto()
    SECURE = auto()
    LIFT = auto()
    HOLD = auto()
    FINISHED = auto()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PyBullet aerial manipulator grasping demo.")
    parser.set_defaults(gui=True, sleep=True, hold_gui=True)
    parser.add_argument("--gui", dest="gui", action="store_true", help="Run the simulation with the PyBullet GUI.")
    parser.add_argument("--headless", dest="gui", action="store_false", help="Run without the PyBullet GUI.")
    parser.add_argument("--urdf", type=str, default=None, help="Optional robot URDF path.")
    parser.add_argument("--sim-steps", type=int, default=7200, help="Maximum number of simulation steps.")
    parser.add_argument("--time-step", type=float, default=1.0 / 240.0, help="PyBullet simulation step size.")
    parser.add_argument("--hover-z", type=float, default=1.25, help="Nominal hover height for the UAV.")
    parser.add_argument("--grasp-z", type=float, default=1.22, help="Lower hover height used during grasping.")
    parser.add_argument("--lift-z", type=float, default=1.38, help="Post-grasp lift height.")
    parser.add_argument("--transit-x", type=float, default=0.55, help="World-frame x position of the pre-grasp hover waypoint.")
    parser.add_argument("--transit-y", type=float, default=0.25, help="World-frame y position of the pre-grasp hover waypoint.")
    parser.add_argument(
        "--target-x",
        type=float,
        default=0.04,
        help="Target x offset relative to the home end-effector pose at the transit waypoint.",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=0.04,
        help="Target y offset relative to the home end-effector pose at the transit waypoint.",
    )
    parser.add_argument(
        "--target-z-offset",
        type=float,
        default=-0.02,
        help="Target z offset relative to the home end-effector pose after hover stabilization.",
    )
    parser.add_argument("--sleep", dest="sleep", action="store_true", help="Play the simulation in real time.")
    parser.add_argument("--fast", dest="sleep", action="store_false", help="Run the simulation as fast as possible.")
    parser.add_argument(
        "--hold-gui",
        dest="hold_gui",
        action="store_true",
        help="Keep the PyBullet window open after the demo finishes.",
    )
    parser.add_argument(
        "--close-on-finish",
        dest="hold_gui",
        action="store_false",
        help="Close the PyBullet window immediately after the demo finishes.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    client_id = p.connect(p.GUI if args.gui else p.DIRECT)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.setTimeStep(args.time_step)
    p.loadURDF("plane.urdf")

    source_urdf = find_robot_urdf(args.urdf)
    prepared_urdf = prepare_urdf_for_pybullet(source_urdf)
    start_position = [0.0, 0.0, max(1.0, args.grasp_z + 0.10)]
    start_orientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])

    robot_id = p.loadURDF(
        prepared_urdf.as_posix(),
        start_position,
        start_orientation,
        useFixedBase=False,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    configure_loaded_robot(p, robot_id)
    transit_hover_target = [args.transit_x, args.transit_y, args.hover_z]

    uav_controller = UAVController(p, robot_id, time_step=args.time_step)
    arm_controller = ArmController(
        p,
        robot_id,
        prepared_urdf,
        ARM_JOINT_NAMES,
        end_effector_link_name="arm_link_5",
    )
    gripper = VirtualGripper(p, robot_id, arm_controller.end_effector_link_index)
    predicted_home_position, _ = arm_controller.predict_end_effector_pose(
        base_position=transit_hover_target,
        base_orientation=start_orientation,
        joint_positions=arm_controller.home_positions,
    )
    target_spawn = predicted_home_position + np.array(
        [args.target_x, args.target_y, args.target_z_offset],
        dtype=float,
    )
    scene = create_demo_scene(p, target_position=tuple(target_spawn.tolist()))

    print(f"Loaded robot URDF: {source_urdf}")
    print(f"Prepared robot URDF for PyBullet: {prepared_urdf}")
    print("Joint index layout:")
    for line in describe_joint_layout(p, robot_id):
        print(f"  {line}")
    print("")
    print("Joint role notes:")
    print("  joint_index 0-3  : rotor placeholder joints from the UAV CAD export, left passive in this demo.")
    print("  joint_index 4    : fixed mount joint from UAV body to arm base.")
    print("  joint_index 5-9  : arm_joint_1 .. arm_joint_5, the five actuated arm joints used by the DLS IK solver.")
    print("")
    print(f"Spawned target at {np.round(target_spawn, 3).tolist()}")
    print(f"Transit waypoint set to {np.round(transit_hover_target, 3).tolist()}")

    state = DemoState.TAKEOFF
    state_entry_step = 0
    hold_steps = 0
    pre_grasp_height = 0.08
    grasp_height = 0.03
    stabilize_steps = 18
    secure_steps = 20
    grasp_ready_steps = 0
    grasp_anchor_position = None
    grasp_end_effector_orientation = None
    secure_target_position = None
    lift_clearance = 0.12
    end_effector_below_base_margin = 0.34
    takeoff_settle_steps = 20
    align_settle_steps = 20

    def compute_side_approach_offsets(target_position_array: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        direction_xy = target_position_array[:2] - np.array([args.transit_x, args.transit_y], dtype=float)
        norm = float(np.linalg.norm(direction_xy))
        if norm < 1e-6:
            unit_xy = np.array([1.0, 0.0], dtype=float)
        else:
            unit_xy = direction_xy / norm

        # Approach from the near side of the object instead of plunging through the pillar centerline.
        side_offset_xy = -0.022 * unit_xy
        pre_grasp_offset = np.array([side_offset_xy[0], side_offset_xy[1], pre_grasp_height], dtype=float)
        grasp_offset = np.array([side_offset_xy[0], side_offset_xy[1], grasp_height], dtype=float)
        return pre_grasp_offset, grasp_offset

    def constrain_grasp_target(target_position_array: np.ndarray, raw_offset: np.ndarray) -> list[float]:
        target_command = target_position_array + raw_offset
        pedestal_top_z = scene.target_spawn_position[2] - 0.025
        target_command[2] = max(target_command[2], pedestal_top_z + 0.045)
        return target_command.tolist()

    try:
        for step in range(args.sim_steps):
            target_position, _ = p.getBasePositionAndOrientation(scene.target_id)
            target_position_array = np.asarray(target_position, dtype=float)
            pre_grasp_offset, grasp_offset = compute_side_approach_offsets(target_position_array)

            if state == DemoState.TAKEOFF:
                hover_target = [0.0, 0.0, args.hover_z]
                uav_controller.step(hover_target)
                arm_controller.command_home()
                if step - state_entry_step >= takeoff_settle_steps and uav_controller.at_target(hover_target):
                    state, state_entry_step = DemoState.TRANSIT, step
                    print(f"[step {step}] Transition -> TRANSIT")

            elif state == DemoState.TRANSIT:
                uav_controller.step(transit_hover_target)
                arm_controller.command_home()
                if uav_controller.at_target(transit_hover_target, position_tolerance=0.05):
                    state, state_entry_step = DemoState.ALIGN, step
                    print(f"[step {step}] Transition -> ALIGN")

            elif state == DemoState.ALIGN:
                uav_controller.step(transit_hover_target)
                arm_controller.command_home()
                if step - state_entry_step >= align_settle_steps and uav_controller.at_target(transit_hover_target, position_tolerance=0.04):
                    state, state_entry_step = DemoState.DESCEND, step
                    print(f"[step {step}] Transition -> DESCEND")

            elif state == DemoState.DESCEND:
                hover_target = [args.transit_x, args.transit_y, args.grasp_z]
                uav_controller.step(hover_target)
                arm_controller.command_home()
                if uav_controller.at_target(hover_target, position_tolerance=0.04):
                    state, state_entry_step = DemoState.APPROACH, step
                    print(f"[step {step}] Transition -> APPROACH")

            elif state == DemoState.APPROACH:
                hover_target = [args.transit_x, args.transit_y, args.grasp_z]
                approach_target = constrain_grasp_target(target_position_array, pre_grasp_offset)
                uav_controller.step(hover_target)
                residual = arm_controller.command_end_effector(approach_target)
                if residual <= 0.03 and arm_controller.at_pose(approach_target, tolerance=0.04):
                    state, state_entry_step = DemoState.STABILIZE, step
                    print(f"[step {step}] Transition -> STABILIZE")

            elif state == DemoState.STABILIZE:
                hover_target = [args.transit_x, args.transit_y, args.grasp_z]
                approach_target = constrain_grasp_target(target_position_array, pre_grasp_offset)
                uav_controller.step(hover_target)
                arm_controller.command_end_effector(approach_target)
                if step - state_entry_step >= stabilize_steps:
                    grasp_ready_steps = 0
                    state, state_entry_step = DemoState.GRASP, step
                    print(f"[step {step}] Transition -> GRASP")

            elif state == DemoState.GRASP:
                hover_target = [args.transit_x, args.transit_y, args.grasp_z]
                grasp_target = constrain_grasp_target(target_position_array, grasp_offset)
                uav_controller.step(hover_target)
                arm_controller.command_end_effector(grasp_target)
                if gripper.is_grasp_ready(scene.target_id):
                    grasp_ready_steps += 1
                else:
                    grasp_ready_steps = 0

                if grasp_ready_steps >= 4:
                    attempt = gripper.close_with_options(
                        scene.target_id,
                        disable_collisions_with=[scene.pedestal_id],
                        release_constraint_ids=[scene.target_support_constraint_id],
                    )
                    if attempt.success:
                        grasp_anchor_position = target_position_array.copy()
                        _, grasp_end_effector_orientation = arm_controller.get_end_effector_pose()
                        secure_target_position = constrain_grasp_target(grasp_anchor_position, grasp_offset)
                        state, state_entry_step = DemoState.SECURE, step
                        print(f"[step {step}] Grasp success ({attempt.reason}). Transition -> SECURE")
                elif step - state_entry_step > 360:
                    raise RuntimeError("Failed to create a grasp constraint within the grasp timeout.")

            elif state == DemoState.SECURE:
                hover_target = [args.transit_x, args.transit_y, args.grasp_z]
                uav_controller.step(hover_target)
                arm_controller.command_end_effector(secure_target_position, target_orientation=grasp_end_effector_orientation)
                if step - state_entry_step >= secure_steps:
                    state, state_entry_step = DemoState.LIFT, step
                    print(f"[step {step}] Transition -> LIFT")

            elif state == DemoState.LIFT:
                lift_progress = min(1.0, (step - state_entry_step) / 60.0)
                hover_target = [args.transit_x, args.transit_y, args.grasp_z + lift_progress * (args.lift_z - args.grasp_z)]
                desired_lift_z = grasp_anchor_position[2] + 0.04 + lift_clearance * lift_progress
                safe_lift_z = min(desired_lift_z, hover_target[2] - end_effector_below_base_margin)
                safe_lift_z = max(safe_lift_z, grasp_anchor_position[2] + 0.03)
                lift_target = [
                    grasp_anchor_position[0],
                    grasp_anchor_position[1],
                    safe_lift_z,
                ]
                uav_controller.step(hover_target)
                arm_controller.command_end_effector(lift_target, target_orientation=grasp_end_effector_orientation)
                if uav_controller.at_target(hover_target, position_tolerance=0.05) and gripper.is_holding_object():
                    state, state_entry_step = DemoState.HOLD, step
                    print(f"[step {step}] Transition -> HOLD")

            elif state == DemoState.HOLD:
                hover_target = [args.transit_x, args.transit_y, args.lift_z]
                safe_hold_z = min(
                    grasp_anchor_position[2] + 0.04 + lift_clearance,
                    hover_target[2] - end_effector_below_base_margin,
                )
                safe_hold_z = max(safe_hold_z, grasp_anchor_position[2] + 0.05)
                hold_target = [
                    grasp_anchor_position[0],
                    grasp_anchor_position[1],
                    safe_hold_z,
                ]
                uav_controller.step(hover_target)
                arm_controller.command_end_effector(hold_target, target_orientation=grasp_end_effector_orientation)
                hold_steps += 1
                if hold_steps >= 240:
                    state = DemoState.FINISHED
                    print(f"[step {step}] Demo finished successfully.")

            elif state == DemoState.FINISHED:
                break

            p.stepSimulation()
            if args.gui and args.sleep:
                time.sleep(args.time_step)

    finally:
        if args.gui and args.hold_gui and p.isConnected(client_id):
            print("Simulation finished. Close the PyBullet window to exit.")
            try:
                while p.isConnected(client_id):
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        arm_controller.shutdown()
        if p.isConnected(client_id):
            p.disconnect(client_id)


if __name__ == "__main__":
    main()
