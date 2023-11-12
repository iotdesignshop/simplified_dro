# simplified_robotics

This is a ROS 2 Package for control of the Simplified Robotics Auto-Mate Arm and other robot arms based on the Interbotix X-Series

The provides a set of User Interface Widgets that makes it easier to program and use the arms in a Python development environment. An overview of the major UI features is presented below.

The [full user manual is available here](https://www.dropbox.com/scl/fi/57d9bquysdey6z5o1n3iv/Simplified-ARC-Manual.odt?rlkey=tzojbvc1ftd9fp0o2nllpdtbd&dl=0)

## Control View

<img width="240" alt="Control-View" src="https://github.com/iotdesignshop/simplified_robotics/assets/2821763/4a8a94e7-00f9-4c0b-ba07-d45fbf2ae848">

The Control View provides a UI for the basic robot state functions.
- **Home**: Wakes the robot and moves it to the standard home pose defined for the robot
- **Sleep**: Returns the robot to it's safe rest pose
- **Torque On/Off**: Used to control motor torque on the robot. Hold for one second to disable motor torque, and single click to turn it back on.

## DRO View

The DRO View provides a "Digital Read Out" of all of the active joint angles and their ranges in addition to load and temperature information on the servos.

It also provides a utility function for copying Python-formatted joint angles (in radians) to the system clipboard so that they can be easily pasted into Python programs.

