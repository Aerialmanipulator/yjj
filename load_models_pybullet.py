import argparse
import math
import tempfile
import time
from pathlib import Path

import pybullet as p
import pybullet_data


WORKSPACE_DIR = Path(__file__).resolve().parent
UAV_URDF = WORKSPACE_DIR / "uav_urdf_export_v1" / "urdf" / "uav_urdf_export_v1.urdf"
ARM_URDF = (
    WORKSPACE_DIR
    / "solidworks_inertia"
    / "arm_urdf_export_v2"
    / "urdf"
    / "arm_urdf_export_v2.urdf"
)
ARM_MOUNT_OFFSET = [0.0, 0.0, -0.12]


def build_pybullet_urdf(source_urdf: Path) -> Path:
    source_urdf = source_urdf.resolve()
    package_root = source_urdf.parent.parent
    package_name = package_root.name
    mesh_prefix = f"package://{package_name}/"
    mesh_root = package_root.as_posix() + "/"

    rewritten_text = source_urdf.read_text(encoding="utf-8").replace(mesh_prefix, mesh_root)
    temp_dir = Path(tempfile.mkdtemp(prefix=f"{package_name}_pybullet_"))
    rewritten_urdf = temp_dir / source_urdf.name
    rewritten_urdf.write_text(rewritten_text, encoding="utf-8")
    return rewritten_urdf


def print_joint_info(body_id: int, body_name: str) -> None:
    joint_count = p.getNumJoints(body_id)
    print(f"\n{body_name} joint count: {joint_count}")
    for joint_index in range(joint_count):
        info = p.getJointInfo(body_id, joint_index)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        lower = info[8]
        upper = info[9]
        print(
            f"  joint[{joint_index}] name={joint_name}, type={joint_type}, "
            f"limit=({lower:.4f}, {upper:.4f})"
        )


def attach_arm_to_uav(uav_id: int, arm_id: int, mount_offset: list[float]) -> int:
    constraint_id = p.createConstraint(
        parentBodyUniqueId=uav_id,
        parentLinkIndex=-1,
        childBodyUniqueId=arm_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0.0, 0.0, 0.0],
        parentFramePosition=mount_offset,
        childFramePosition=[0.0, 0.0, 0.0],
        parentFrameOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
        childFrameOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
    )
    p.changeConstraint(constraint_id, maxForce=500.0)
    return constraint_id


def animate_models(uav_id: int, arm_id: int, steps: int, time_step: float) -> None:
    uav_joint_count = p.getNumJoints(uav_id)
    arm_joint_count = p.getNumJoints(arm_id)

    for step in range(steps):
        t = step * time_step

        # SolidWorks exports the joints with zero limits, so we drive the
        # preview directly with resetJointState for visualization.
        for joint_index in range(uav_joint_count):
            rotor_angle = 25.0 * t * (1 if joint_index % 2 == 0 else -1)
            p.resetJointState(uav_id, joint_index, rotor_angle)

        for joint_index in range(arm_joint_count):
            phase = t * 1.4 + joint_index * 0.55
            arm_angle = 0.45 * math.sin(phase)
            p.resetJointState(arm_id, joint_index, arm_angle)

        p.stepSimulation()
        if p.getConnectionInfo()["connectionMethod"] == p.GUI:
            time.sleep(time_step)


def main() -> None:
    parser = argparse.ArgumentParser(description="Load the UAV and arm URDF models into PyBullet.")
    parser.add_argument("--headless", action="store_true", help="Run in DIRECT mode without the GUI.")
    parser.add_argument("--steps", type=int, default=2400, help="Number of simulation steps to run.")
    parser.add_argument("--time-step", type=float, default=1.0 / 240.0, help="Simulation time step.")
    parser.add_argument(
        "--arm-offset-z",
        type=float,
        default=ARM_MOUNT_OFFSET[2],
        help="Vertical mounting offset from the UAV body to the arm base.",
    )
    args = parser.parse_args()

    connection_mode = p.DIRECT if args.headless else p.GUI
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet.")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.setTimeStep(args.time_step)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.loadURDF("plane.urdf")

    uav_urdf = build_pybullet_urdf(UAV_URDF)
    arm_urdf = build_pybullet_urdf(ARM_URDF)

    uav_id = p.loadURDF(
        str(uav_urdf),
        basePosition=[0.0, 0.0, 0.35],
        baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
        useFixedBase=False,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    arm_id = p.loadURDF(
        str(arm_urdf),
        basePosition=[0.0, 0.0, 0.35 + args.arm_offset_z],
        baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
        useFixedBase=False,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    attach_arm_to_uav(uav_id, arm_id, [0.0, 0.0, args.arm_offset_z])

    if connection_mode == p.GUI:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45.0,
            cameraPitch=-25.0,
            cameraTargetPosition=[0.0, 0.0, 0.2],
        )

    print_joint_info(uav_id, "uav_urdf_export_v1")
    print_joint_info(arm_id, "arm_urdf_export_v2")

    animate_models(uav_id, arm_id, steps=args.steps, time_step=args.time_step)

    if args.headless:
        print("\nHeadless simulation finished.")
        p.disconnect()
        return

    print("\nSimulation finished. Close the PyBullet window to exit.")
    while p.isConnected():
        time.sleep(0.1)


if __name__ == "__main__":
    main()
