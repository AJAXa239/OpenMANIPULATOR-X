#!/usr/bin/env python3
"""
Start MoveIt (move_group + RViz) for the OMX arm.

Run the MuJoCo sim FIRST (it provides controllers, /joint_states, /clock, TF):
    ros2 launch omx_mujoco sim.launch.py

Then, in another terminal:
    ros2 launch omx_moveit_config moveit.launch.py

In RViz: MotionPlanning panel -> drag the orange goal marker or pick a named
state (init/home, open/close) -> Plan -> Execute. The arm moves in MuJoCo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim = LaunchConfiguration("use_sim")

    # robot description comes from the omx_mujoco package (single source of truth)
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
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim, "publish_robot_description_semantic": True},
        ],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("omx_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(start_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim", default_value="true"),
        move_group_node,
        rviz_node,
    ])
