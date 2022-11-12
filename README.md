# ODrive Controller
## Installation
- In order to install ROS2 framework you should follow the guidelines in [Link](https://docs.ros.org/en/humble/Installation.html)
- Install ROS2 dependency, run the following command on terminal in order to install control, controller, xacro and libusb
```
sudo apt-get install libusb-1.0-0-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```
- Download project: Clone git project from [github](https://github.com/nadavis/ODrive_Controller.git) to my project directory
- cd ~/my project
- Run in terminal ```colcon build```
## ROS2 Control Simulation
The simulation is based on Rviz only, the pipline does not attached to ODrive
### DiffBot Demo
[Link](https://github.com/ros-controls/ros2_control_demos)
Differential Mobile Robot is a simple mobile base with differential drive.
1. To check that DiffBot description is working properly use following launch commands:
   ```
   ros2 launch diffbot_description view_robot.launch.py
   ```
2. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with:
   ```
   ros2 launch ros2_control_demo_bringup diffbot.launch.py
   ```
3. Check if the hardware interface loaded properly, by opening another terminal and executing:
   ```
   ros2 control list_hardware_interfaces
   ```
   You should get:
   ```
   command interfaces
        left_wheel_joint/velocity [claimed]
        right_wheel_joint/velocity [claimed]
   state interfaces
         left_wheel_joint/position
         left_wheel_joint/velocity
         right_wheel_joint/position
         right_wheel_joint/velocity
   ```
   The `[claimed]` marker on command interfaces means that a controller has access to command *DiffBot*.

4. Check if controllers are running:
   ```
   ros2 control list_controllers
   ```
   You should get:
   ```
   diffbot_base_controller[diff_drive_controller/DiffDriveController] active
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   ```
5. If everything is fine, now you can send a command to *Diff Drive Controller* using ros2 cli interface:
   ```
   ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
    x: 0.7
    y: 0.0
    z: 0.0
   angular:
    x: 0.0
    y: 0.0
    z: 1.0"
    ```
   You should now see an orange box circling in `RViz`.
   Also, you should see changing states in the terminal where launch file is started.

## ODrive demo 
### ODrive setup
A description [Link](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/Getting_Started)

We are using firmware version 0.5.1, pip instalation:
```
sudo pip3 install odrive==0.5.1.post0
```
### ODrive calibration
#### ODrive terminal basic commands
```
odrivetool
odrv0.erase_configuration()
odrv0.save_configuration()
odrv0.reboot()
dump_errors(odrv0)
```
- Setup the motor and encoder parameters the file run calibrate as well
```
python3 src/odrive/odrive_calibration/two_motors_calibration.py
```
- Motor and encoder calibration and small test
```
python3 src/odrive/odrive_calibration/two_motors_calibration.py
```
### Test ODrive
The following test is using ODrive hardware, so be sure you calibrated the motors and encoders
- Run
```
ros2 launch odrive_demo_bringup odrive_multi_interface.launch.py
```
- Publish topic in new terminal 
```
ros2 topic pub -r 100 /joint0_velocity_controller/commands std_msgs/Float64MultiArray "data: [1]"
```
## ODrive DiffBot 
[Link](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/DiffBot_HIL_Demo)
```
ros2 launch odrive_demo_bringup odrive_diffbot.launch.py
```
Run the following command
```
ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.2
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"
 ```

### ROS2 Control with Joystick
Control Rviz and ODrive with XBox Joystick
- Assuming you hae installed ROS2 Desktop with dependency of joy_node and teleop_twist_joy
- In order ot Enable / Disable hardware or simulation change the values false (with ODrive hardware) / true (without ODrvie) 
```
<xacro:arg name="use_fake_hardware" default="false" />
<xacro:arg name="fake_sensor_commands" default="false" />
```
- Motors devices 
```
ros2 launch rosbot_bringup mini_robot.launch.py
```
- Server 
```
ros2 launch rosbot_bringup mini_rviz.launch.py
```
Run the following command
```
ros2 topic pub --rate 30 /mini_robot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.2
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"
 ```