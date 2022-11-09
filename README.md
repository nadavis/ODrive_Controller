# ODrive Controller
## Installation
- In order to install ROS2 framework you should follow the guidelines in [Link](https://docs.ros.org/en/humble/Installation.html)
- Install ROS2 dependency, run the following command on terminal in order to install control, controller, xacro and libusb
`sudo apt-get install libusb-1.0-0-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro`
- Download project: Clone git project from [github](https://github.com/nadavis/ODrive_Controller.git) to my project directory
- cd ~/my project
- Run in terminal `colcon build`

## Run 
### Odrive calibration
- Run `python3 src/odrive/two_motors_calibration.py`

### Launch ROS - Joystick
Control Odrive with XBox Joystick
- Assuming you hae installed ROS2 Desktop with dependency of joy_node and teleop_twist_joy
- Motors devices `ros2 launch rosbot_bringup mini_robot.launch.py`
- Server `ros2 launch rosbot_bringup mini_rviz.launch.py`