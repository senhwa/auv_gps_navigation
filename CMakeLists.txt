cmake_minimum_required(VERSION 3.5)
project(auv_gps_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install dependencies
find_program(PIP_EXECUTABLE pip3)
if(NOT PIP_EXECUTABLE)
  message(FATAL_ERROR "pip3 not found. Please install it to proceed.")
endif()

# Install Python dependencies
execute_process(
  COMMAND ${PIP_EXECUTABLE} install simple-pid
  RESULT_VARIABLE pid_result
)

if(NOT pid_result EQUAL 0)
  message(FATAL_ERROR "Failed to install simple-pid via pip3. Please check your Python environment.")
endif()

execute_process(
  COMMAND ${PIP_EXECUTABLE} install pyproj
  RESULT_VARIABLE pyproj_result
)

if(NOT pyproj_result EQUAL 0)
  message(FATAL_ERROR "Failed to install pyproj via pip3. Please check your Python environment.")
endif()

# Install Python scripts and other files
install(
  DIRECTORY
    launch
    config
    rviz
    urdf
    meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auv_gps_navigation_node.py
    scripts/navigation_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

# Mark the ament package
ament_package()