#!/usr/bin/env python3
"""
Bring up the OpenMANIPULATOR-X in MuJoCo under ros2_control.

    ros2 launch omx_mujoco sim.launch.py
    ros2 launch omx_mujoco sim.launch.py headless:=true

Starts:
  - robot_state_publisher  (TF from the URDF)
  - mujoco_ros2_control ros2_control_node  (MuJoCo physics + controller_manager)
  - spawners for joint_state_broadcaster, arm_controller, gripper_controller

On every launch the three objects' COLOURS (red/green/blue, one each) and their
POSITIONS are shuffled, so each run starts differently. Positions are drawn from
a set of spots that are all reachable, on the table, and kept well separated.
The vision node detects wherever they land, so sorting still works automatically.
"""
import os
import re
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare

# pick-area spots (x, y), all verified reachable + table-clearing + separated
SAFE_SPOTS = [
    (0.17, -0.09), (0.17, -0.045), (0.17, 0.0), (0.17, 0.045), (0.17, 0.09),
    (0.19, -0.09), (0.19, -0.045), (0.19, 0.0), (0.19, 0.045), (0.19, 0.09),
    (0.21, -0.09), (0.21, -0.045), (0.21, 0.0), (0.21, 0.045), (0.21, 0.09),
]
OBJ_Z = 0.0625        # resting height on the table
MIN_SEP = 0.05        # keep chosen spots at least this far apart


def _choose_positions():
    """Pick 3 spots that are pairwise at least MIN_SEP apart."""
    spots = SAFE_SPOTS[:]
    random.shuffle(spots)
    for _ in range(200):
        random.shuffle(spots)
        chosen = []
        for s in spots:
            if all((s[0] - c[0]) ** 2 + (s[1] - c[1]) ** 2 >= MIN_SEP ** 2 for c in chosen):
                chosen.append(s)
            if len(chosen) == 3:
                return chosen
    return spots[:3]      # fallback (shouldn't happen)


def _randomize_scene():
    """Shuffle the three objects' colours AND positions in the installed scene.xml."""
    try:
        scene = os.path.join(
            get_package_share_directory("omx_mujoco"), "mujoco", "scene.xml"
        )
        xml = open(scene).read()
        bodies = ["cube", "cylinder", "hexagon"]
        geoms = ["cube_geom", "cyl_geom", "hex_geom"]
        mats = ["red_obj", "green_obj", "blue_obj"]
        random.shuffle(mats)
        pos = _choose_positions()

        for b, g, m, (x, y) in zip(bodies, geoms, mats, pos):
            # colour: set this object's geom material
            xml = re.sub(r'(name="%s"[^>]*?material=")[a-z_]+_obj(")' % g,
                         r"\g<1>%s\g<2>" % m, xml, count=1)
            # position: set this object's body pos
            xml = re.sub(r'(<body name="%s" pos=")[^"]*(")' % b,
                         r"\g<1>%.3f %.3f %.4f\g<2>" % (x, y, OBJ_Z), xml, count=1)
        open(scene, "w").write(xml)
        print("[sim.launch] randomized: " +
              ", ".join(f"{b}={m}@({x:.2f},{y:.2f})"
                        for b, m, (x, y) in zip(bodies, mats, pos)))
    except Exception as e:   # never let randomization stop the sim from launching
        print(f"[sim.launch] scene randomization skipped: {e}")


def launch_setup(context, *args, **kwargs):
    _randomize_scene()      # <-- shuffle colours + positions before MuJoCo loads the scene

    pkg = FindPackageShare("omx_mujoco")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg, "urdf", "omx.urdf.xacro"]),
        " headless:=",
        LaunchConfiguration("headless"),
    ])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content.perform(context), value_type=str
        )
    }

    controllers_file = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    nodes = []

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    ))

    nodes.append(Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[{"use_sim_time": True}, ParameterFile(controllers_file)],
        remappings=(
            [("~/robot_description", "/robot_description")]
            if os.environ.get("ROS_DISTRO") == "humble" else []
        ),
        on_exit=Shutdown(),
    ))

    for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--param-file", controllers_file],
            output="both",
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run MuJoCo without the Simulate window",
        ),
        OpaqueFunction(function=launch_setup),
    ])
