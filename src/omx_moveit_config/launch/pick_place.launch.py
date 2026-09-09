#!/usr/bin/env python3
"""
Run the moveit_py pick-and-place node.

Prereq (other terminals):
    ros2 launch omx_mujoco sim.launch.py         # physics + controllers
    ros2 launch omx_moveit_config moveit.launch.py   # optional: RViz view
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    omx_xacro = os.path.join(
        get_package_share_directory("omx_mujoco"), "urdf", "omx.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="omx_moveit_config")
        .robot_description(file_path=omx_xacro, mappings={"headless": "true"})
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .moveit_cpp(file_path="config/moveit_py.yaml")
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="omx_moveit_config",
        executable="pick_and_place.py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_node])
