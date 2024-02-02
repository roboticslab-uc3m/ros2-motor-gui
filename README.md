# ROS2 TEO Workspace

ROS2 Motor GUI for TEO robot, an easy way to send basic movement commands to TEO using ROS2 through an interface.

## Installation

Install the ROS2 Motor Gui workspace

```bash
  git clone https://github.com/roboticslab-uc3m/ros2-motor-gui.git
  cd ros2-motor-gui
  source /opt/ros/humble/setup.bash
  colcon build
```

## Usage/Examples

After installing:
```bash
source install/setup.bash
ros2 run py_motor_gui motorGui
```

You should see something like this:
![motor-gui](https://github.com/roboticslab-uc3m/ros2-motor-gui/assets/38068010/c4132cb4-9481-430c-9553-ac9e71b27a8b)

It's pretty easy to use, just select the articulation that you want to move and set the position (degrees) and velocity (radians).
![ezgif-4-2bb29d81f8](https://github.com/roboticslab-uc3m/ros2-motor-gui/assets/38068010/e9b41cc4-f79c-412c-aa88-ef7f7a41cd40)
