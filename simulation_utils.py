from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
GENERATED_DIR = PROJECT_ROOT / "generated"

# The workspace currently contains a typoed file name (`fianl.urdf`).
# We support the user-requested names first and fall back to the existing file.
DEFAULT_URDF_CANDIDATES = (
    PROJECT_ROOT / "urdf" / "final.urdf2",
    PROJECT_ROOT / "urdf" / "final.urdf",
    PROJECT_ROOT / "urdf" / "fianl.urdf",
)

PACKAGE_MAP = {
    "uav_urdf_export_v1": PROJECT_ROOT / "project" / "uav_urdf_export_v1" / "uav_urdf_export_v1",
    "arm_urdf_export_v2": PROJECT_ROOT / "project" / "solidworks_inertia" / "solidworks_inertia" / "arm_urdf_export_v2",
}

# Temporary arm joint limits for simulation.
# The source URDF currently exports all arm joints with zero-width limits.
ARM_JOINT_LIMITS = {
    "arm_joint_1": (-3.14159, 3.14159, 80.0, 2.5),
    "arm_joint_2": (-2.6, 2.6, 80.0, 2.5),
    "arm_joint_3": (-2.6, 2.6, 80.0, 2.5),
    "arm_joint_4": (-3.14159, 3.14159, 40.0, 3.0),
    "arm_joint_5": (-3.14159, 3.14159, 20.0, 4.0),
}

ROTOR_JOINT_NAMES = ("joint0", "joint1", "joint2", "joint3")
ARM_JOINT_NAMES = tuple(ARM_JOINT_LIMITS.keys())


@dataclass(frozen=True)
class DemoScene:
    pedestal_id: int
    target_id: int
    target_spawn_position: tuple[float, float, float]
    target_support_constraint_id: int | None


def find_robot_urdf(explicit_path: str | Path | None = None) -> Path:
    if explicit_path:
        candidate = Path(explicit_path)
        if not candidate.is_absolute():
            candidate = PROJECT_ROOT / candidate
        if candidate.exists():
            return candidate.resolve()
        raise FileNotFoundError(f"Robot URDF not found: {candidate}")

    for candidate in DEFAULT_URDF_CANDIDATES:
        if candidate.exists():
            return candidate.resolve()

    searched = ", ".join(str(path) for path in DEFAULT_URDF_CANDIDATES)
    raise FileNotFoundError(f"Could not find a robot URDF. Searched: {searched}")


def _resolve_package_uri(uri: str) -> str:
    if not uri.startswith("package://"):
        return uri

    package_path = uri.removeprefix("package://")
    package_name, _, relative_path = package_path.partition("/")
    package_root = PACKAGE_MAP.get(package_name)
    if package_root is None:
        raise KeyError(f"Unsupported package URI: {uri}")

    resolved = (package_root / relative_path).resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Resolved mesh path does not exist: {resolved}")
    return resolved.as_posix()


def prepare_urdf_for_pybullet(source_urdf: str | Path, output_dir: str | Path | None = None) -> Path:
    source_path = Path(source_urdf).resolve()
    target_dir = Path(output_dir) if output_dir else GENERATED_DIR
    target_dir.mkdir(parents=True, exist_ok=True)
    output_path = target_dir / f"{source_path.stem}_pybullet.urdf"

    tree = ET.parse(source_path)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", _resolve_package_uri(filename))

    for joint in root.findall("joint"):
        joint_name = joint.get("name", "")
        if joint_name not in ARM_JOINT_LIMITS:
            continue

        lower, upper, effort, velocity = ARM_JOINT_LIMITS[joint_name]
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")

        limit.set("lower", str(lower))
        limit.set("upper", str(upper))
        limit.set("effort", str(effort))
        limit.set("velocity", str(velocity))

        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set("damping", "0.08")
        dynamics.set("friction", "0.02")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def configure_loaded_robot(pb, robot_id: int) -> None:
    for link_index in range(-1, pb.getNumJoints(robot_id)):
        pb.changeDynamics(
            robot_id,
            link_index,
            linearDamping=0.04,
            angularDamping=0.04,
        )

    for joint_index in range(pb.getNumJoints(robot_id)):
        joint_info = pb.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]

        if joint_name in ROTOR_JOINT_NAMES:
            # Joint indices 0-3 are the four rotor placeholder joints from the CAD export.
            # They are kept passive in this kinematic manipulation demo.
            pb.setJointMotorControl2(
                robot_id,
                joint_index,
                pb.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
            )
        elif joint_type == pb.JOINT_FIXED:
            continue
        elif joint_name not in ARM_JOINT_NAMES:
            pb.setJointMotorControl2(
                robot_id,
                joint_index,
                pb.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
            )


def describe_joint_layout(pb, robot_id: int) -> list[str]:
    lines: list[str] = []
    for joint_index in range(pb.getNumJoints(robot_id)):
        joint_info = pb.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        child_link_name = joint_info[12].decode("utf-8")
        joint_type = joint_info[2]
        type_name = {
            pb.JOINT_REVOLUTE: "revolute",
            pb.JOINT_PRISMATIC: "prismatic",
            pb.JOINT_FIXED: "fixed",
        }.get(joint_type, str(joint_type))
        lines.append(
            f"joint_index={joint_index:02d} name={joint_name:<16} type={type_name:<9} child_link={child_link_name}"
        )
    return lines


def create_demo_scene(
    pb,
    target_position: tuple[float, float, float] = (0.02, 0.03, 0.63),
    pedestal_half_extents_xy: tuple[float, float] = (0.06, 0.06),
) -> DemoScene:
    target_half_extents = [0.025, 0.025, 0.025]
    support_top_z = max(target_position[2] - target_half_extents[2], 0.04)
    pedestal_half_extents = [pedestal_half_extents_xy[0], pedestal_half_extents_xy[1], support_top_z / 2.0]
    pedestal_collision = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=pedestal_half_extents)
    pedestal_visual = pb.createVisualShape(
        pb.GEOM_BOX,
        halfExtents=pedestal_half_extents,
        rgbaColor=[0.65, 0.65, 0.68, 1.0],
    )
    pedestal_position = [target_position[0], target_position[1], pedestal_half_extents[2]]
    pedestal_id = pb.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=pedestal_collision,
        baseVisualShapeIndex=pedestal_visual,
        basePosition=pedestal_position,
    )
    pb.changeDynamics(
        pedestal_id,
        -1,
        lateralFriction=1.5,
        spinningFriction=0.02,
        rollingFriction=0.02,
        restitution=0.0,
    )

    target_collision = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=target_half_extents)
    target_visual = pb.createVisualShape(
        pb.GEOM_BOX,
        halfExtents=target_half_extents,
        rgbaColor=[0.90, 0.15, 0.15, 1.0],
    )
    target_id = pb.createMultiBody(
        baseMass=0.08,
        baseCollisionShapeIndex=target_collision,
        baseVisualShapeIndex=target_visual,
        basePosition=list(target_position),
    )

    pb.changeDynamics(
        target_id,
        -1,
        lateralFriction=1.3,
        rollingFriction=0.01,
        spinningFriction=0.01,
        linearDamping=0.10,
        angularDamping=0.14,
        restitution=0.0,
    )
    target_support_constraint_id = pb.createConstraint(
        pedestal_id,
        -1,
        target_id,
        -1,
        pb.JOINT_FIXED,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, pedestal_half_extents[2]],
        [0.0, 0.0, -target_half_extents[2]],
    )
    return DemoScene(
        pedestal_id=pedestal_id,
        target_id=target_id,
        target_spawn_position=target_position,
        target_support_constraint_id=target_support_constraint_id,
    )
