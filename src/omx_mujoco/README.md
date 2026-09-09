# omx_mujoco — OpenMANIPULATOR-X in MuJoCo under ros2_control

Step 2 of the MoveIt pick-and-place build. This runs the real OMX 4-DOF arm
inside MuJoCo, driven through **`mujoco_ros2_control`**, with a
`joint_trajectory_controller` on the arm and one on the gripper — exactly the
controllers MoveIt will command in Step 3. No MoveIt yet; this step proves the
arm + controllers work.

```
omx_mujoco/
├── mujoco/
│   ├── omx.xml         # robot MJCF: real meshes, joint-named actuators, gripper tendon
│   ├── scene.xml       # wraps omx.xml + floor/lights (this is 'mujoco_model')
│   └── assets/*.stl    # real ROBOTIS meshes
├── urdf/omx.urdf.xacro # OMX kinematics + <ros2_control> block (TF + MoveIt)
├── config/controllers.yaml
├── launch/sim.launch.py
├── scripts/test_motion.py
├── CMakeLists.txt
└── package.xml
```

## 1. Put it in your workspace and build

```bash
# copy this folder into your ROS 2 workspace
cp -r omx_mujoco ~/ros2_ws/src/

cd ~/ros2_ws
# controllers (usually already present with ros2_control)
sudo apt install ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster ros-jazzy-xacro

colcon build --packages-select omx_mujoco
source install/setup.bash
```

## 2. Launch the simulation

```bash
ros2 launch omx_mujoco sim.launch.py
```

You should get a MuJoCo window with the OMX arm standing on the floor, and in
the log: `MujocoSystem` loaded, then `joint_state_broadcaster`,
`arm_controller`, and `gripper_controller` all activated.

Check controllers are active (new terminal, `source install/setup.bash` first):

```bash
ros2 control list_controllers
# arm_controller       ... active
# gripper_controller   ... active
# joint_state_broadcaster ... active
```

## 3. Test motion (no MoveIt)

```bash
ros2 run omx_mujoco test_motion.py
```

The arm runs a short wave and the gripper opens/closes in the MuJoCo window.
That confirms trajectories flow: ROS → controllers → MuJoCo.

## Notes

- The gripper is driven by one command joint (`gripper_left_joint`); the right
  finger has no actuator and is kept mirrored by a MuJoCo tendon + equality
  constraint — this matches how `mujoco_ros2_control` expects mimic grippers.
- Physics live in `scene.xml`; the URDF only provides kinematics for TF and
  (next) MoveIt. Joint names match between the two, so TF and physics agree.
- This package supersedes the standalone `~/mujoco_ws/omx_mujoco` from Phase 1.

## Next (Step 3)

Add MoveIt: reuse ROBOTIS's official `open_manipulator_x` SRDF / kinematics /
OMPL config, point its controllers at `arm_controller` + `gripper_controller`,
launch `move_group` + RViz, then a `moveit_py` pick-and-place node.
