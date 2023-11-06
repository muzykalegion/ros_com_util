# PX4-ROS2 Utility nodes

 1. Sensor combined IMU listener 
 
    Listener of `/fmu/out/sensor_combined` topic converts IMU data and publishes it as `sensor_msgs/msg/Imu` message into `/fmu/out/imu` topic

## Install, build and usage

Build `colcon build`

Install `source install/setup.bash`

Run `ros2 launch ros_com_util sensor_combined_listener.launch.py`

## Questions and troubleshooting

[Mailto author](mailto:muzyka.legion@gmail.com)

