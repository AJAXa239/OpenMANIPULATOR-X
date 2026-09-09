# omx_moveit_config — MoveIt 2 for the OMX MuJoCo arm

Step 3 of the pick-and-place build: MoveIt plans for the OpenMANIPULATOR-X and
executes on the running `omx_mujoco` MuJoCo simulation.

Reuses ROBOTIS's official `open_manipulator_x` SRDF, kinematics (KDL,
position-only IK for the 4-DOF arm), joint limits, and OMPL config. The only
change is `moveit_controllers.yaml`: the gripper is mapped as
`FollowJointTrajectory` to match our `gripper_controller` (a JTC). Robot
description is pulled from the `omx_mujoco` package, so there is one source of
truth for the URDF.

## Build

```bash
cp -r omx_moveit_config ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select omx_moveit_config
source install/setup.bash
```

## Run (two terminals)

Terminal 1 — the simulation (controllers, /joint_states, /clock, TF):
```bash
ros2 launch omx_mujoco sim.launch.py
```

Terminal 2 — MoveIt + RViz:
```bash
source install/setup.bash
ros2 launch omx_moveit_config moveit.launch.py
```

In the RViz **MotionPlanning** panel:
- Planning tab -> set a goal (drag the interactive marker, or choose a named
  state: `init` / `home` for the arm, `open` / `close` for the gripper).
- **Plan**, then **Execute** — the arm moves in the MuJoCo window.

## Notes

- Start the sim first; `move_group` uses sim time and waits for `/clock`.
- Execution goes to `/arm_controller/follow_joint_trajectory` and
  `/gripper_controller/follow_joint_trajectory` (both JTC action servers from
  the sim).

## Next

Add a table + cube to `omx_mujoco/mujoco/scene.xml` (and as MoveIt collision
objects), then a `moveit_py` pick-and-place node.
