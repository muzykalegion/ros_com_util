# PX4-ROS2 Utility nodes

 1. Sensor combined IMU listener 
 
    Listener of `/fmu/out/sensor_combined` topic converts IMU data and publishes it as `sensor_msgs/msg/Imu` message into `/fmu/out/imu` topic

    This package has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package.

## Install, build and usage

Build `colcon build`

Install `source install/setup.bash`

Run `ros2 launch ros_com_util sensor_combined_listener.launch.py`

## Questions and troubleshooting

[Mailto author](mailto:muzyka.legion@gmail.com)

