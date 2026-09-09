cmake_minimum_required(VERSION 3.8)
project(omx_mujoco)

find_package(ament_cmake REQUIRED)

# resource directories
install(
  DIRECTORY urdf mujoco config launch
  DESTINATION share/${PROJECT_NAME}
)

# python test node -> runnable with `ros2 run omx_mujoco test_motion.py`
install(
  PROGRAMS scripts/test_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
