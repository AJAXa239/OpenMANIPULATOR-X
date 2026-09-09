#!/usr/bin/env python3
"""
Sanity test for the OMX ros2_control setup (no MoveIt yet).

Sends two trajectories:
  1. a small "wave" on the 4 arm joints via /arm_controller
  2. open then close on /gripper_controller

Run AFTER `ros2 launch omx_mujoco sim.launch.py` is up:

    ros2 run omx_mujoco test_motion.py
    # or:  python3 test_motion.py
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINTS = ["gripper_left_joint"]


def point(positions, sec):
    p = JointTrajectoryPoint()
    p.positions = [float(x) for x in positions]
    p.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
    return p


class Tester(Node):
    def __init__(self):
        super().__init__("omx_test_motion")
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.timer = self.create_timer(1.0, self.run_once)
        self.done = False

    def run_once(self):
        if self.done:
            return
        self.done = True

        arm = JointTrajectory()
        arm.joint_names = ARM_JOINTS
        arm.points = [
            point([0.0, 0.0, 0.0, 0.0], 1.0),
            point([0.8, 0.5, -0.4, 0.3], 3.0),
            point([-0.8, 0.3, -0.2, 0.6], 5.0),
            point([0.0, 0.0, 0.0, 0.0], 7.0),
        ]
        self.arm.publish(arm)
        self.get_logger().info("Sent arm wave trajectory (7 s).")

        grip = JointTrajectory()
        grip.joint_names = GRIPPER_JOINTS
        grip.points = [
            point([0.019], 2.0),    # open
            point([-0.010], 4.0),   # close
            point([0.019], 6.0),    # open
        ]
        self.grip.publish(grip)
        self.get_logger().info("Sent gripper open/close trajectory.")


def main():
    rclpy.init()
    node = Tester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
