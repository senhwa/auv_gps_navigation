# AUV GPS Navigation

`auv_gps_navigation` is a ROS2 package for autonomous underwater vehicle (AUV) navigation using GPS and IMU data. This package helps the AUV follow designated waypoints and uses PID control to reach the target positions.

## Key Features
- Calculates the current position of the AUV using GPS and IMU data.
- Sequentially follows waypoints, moving to the next one once a waypoint is reached.
- Uses PID controllers to adjust the AUV's yaw direction and x-axis thrust.
- Issues warnings when the GPS signal is lost, and resumes navigation when the signal is restored.

## System Requirements
- ROS2 Humble version
- Python 3.8 or later
- `simple_pid` Python package (automatically installed via CMake)
- sudo apt-get install ros-humble-tf-transformations

## Installation
Clone and build the package using the following commands:

    cd ~/ros_ws/src
    git clone <repository_url>
    cd ~/ros_ws
    colcon build --packages-select auv_gps_navigation

### Dependency Installation
ROS2 and Python dependencies are automatically installed via CMake. If `simple_pid` is not available, CMake will automatically install it using `pip`.

## Running the Node
Once the build is complete, run the node using the following commands:

    source ~/ros_ws/install/setup.bash
    ros2 run auv_gps_navigation auv_navigation_node.py

## File Structure
    src/
      auv_gps_navigation/
        CMakeLists.txt      # Build and installation configuration file
        package.xml         # Package metadata
        launch/             # Launch files (planned)
        config/             # Configuration files (planned)
        scripts/
          auv_navigation_node.py  # AUV navigation node Python script

## Node Description
### `auv_navigation_node.py`
This node performs the following functions:
- Subscribes to GPS, IMU, and depth data topics (`/sensing/gps/data`, `/sensing/imu/data`, `/sensing/depth/data`).
- Publishes the current position of the AUV as a `geometry_msgs/PoseStamped` message and broadcasts TF using `tf2_ros`.
- Publishes `geometry_msgs/Wrench` messages to control the AUV's thrust to move towards the given waypoints.
- Stops the AUV and issues a warning message if the GPS signal is not received for a certain period.

## Main Topics
- `/sensing/gps/data` (`sensor_msgs/NavSatFix`): Subscribes to GPS data.
- `/sensing/imu/data` (`sensor_msgs/Imu`): Subscribes to IMU data.
- `/sensing/depth/data` (`std_msgs/Float32`): Subscribes to depth data.
- `/auv/pose` (`geometry_msgs/PoseStamped`): Publishes the current position of the AUV.
- `/auv/wrench` (`geometry_msgs/Wrench`): Publishes commands to control the AUV's thrust.

## License
This project is licensed under the Apache 2.0 License.

## Contributing
Contributions are welcome! Please create an issue for bug reports or feature requests.