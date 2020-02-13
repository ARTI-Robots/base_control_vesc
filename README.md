# base_control_vesc

[ROS](http://wiki.ros.org/) library and node for controlling a robot base (an automated ground vehicle).
This library assumes that the robot uses the [VESC](https://vesc-project.com/) motor
controller.

Takes Twist and Ackermann messages and controls motors accordingly. In order to calculate the motor 
commands the library uses the [base control](https://github.com/ARTI-Robots/base_control) library

This repository contains the following packages:

- *arti_base_control_vesc* contains the library as plugin to be used in the base contro node.

## Prerequisites

- ROS Kinetic
- [vesc repository](https://github.com/ARTI-Robots/vesc)
- [base control](https://github.com/ARTI-Robots/base_control)

## License

All files in this repository are distributed under the terms of the 2-clause BSD license. See `LICENSE.txt` for
details.
