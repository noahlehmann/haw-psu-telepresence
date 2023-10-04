# Telepresence Robot Project
## Introduction
This repository documents a joint project between Hof University, Germany, and PSU Abington, Philadelphia. The aim is to develop a telepresence robot built upon the Turtle Bot 3 platform.

### Objectives
1. **Turtle Bot Setup:** Equip the Turtle Bot with a compatible operating system, integrated with the Robot Operating System (ROS).
2. **Remote Control**: Develop scripts that allow users to control the robot remotely.
3. **Hardware Add-ons:** Design and 3D print a mobile phone holder to use the smartphone as the robot's camera and screen.
### Minimum Viable Product (MVP)
The project's MVP is a robot capable of being controlled remotely and facilitating communication via Zoom through the attached smartphone.


## Setting Up the Telepresence Robot

### Initial Setup

Follow this short guide to set up your robot. The steps have been tested on Bot 2. Bot 3 has been cloned from Bot 2 but remains untested.

1. Flash Ubuntu 20.04 onto the robot.
2. **Important**: For steps 3 and 4, make sure to select "Noetic" at the top of the linked website to ensure compatibility with the correct version of ROS.
3. Set up the PC following the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).
4. Configure OpenCR by following the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup).
5. **Critical Step**: Shutdown Ubuntu. Disconnect the USB power connection. Insert a charged battery. Turn on the switch located on the lower PCB. The robot should boot up. If you hear beeping sounds, the battery is empty, and the motors will not function.
6. Prepare to open two terminal sessions. Commands for each terminal are provided below.

### Running the Control Scripts

After the setup is complete, you can proceed to control the robot using the following scripts:

#### Terminal 1:
This terminal initializes the core functionalities and links the Turtle Bot to the ROS setup.
```bash
source /opt/ros/noetic/setup.bash
roslaunch turtlebot3_bringup turtlebot3_core.launch
```
#### Terminal 2:
This terminal captures keystrokes and sends them to the ROS instance running in Terminal 1, allowing you to control the robot's movement

```bash
source /opt/ros/noetic/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

With these commands, you can now achieve basic remote control of the robot.
