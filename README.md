# Vention Rail ROS 2 Drivers

ROS 2 control hardware interface for the Vention Linear Rail robot.
Built and tested against ROS 2 humble.

## Overview

In the initialization stage of the hardware interface, the rail homes itself, which moves all the way to the motor side of the rail.
To make sure that as the rail homes, it will not collide with anything throughout the move, we have set up an interface that requires an operator to press a button located underneath the driver computer.
When the hardware interface starts, it will prompt the user with the following message (which will be mixed with other controller messages waiting for hardware interfaces to start).

```bash
PLEASE PRESS THE SAFETY BUTTON TO START HOMING
```

An operator should walk to the robot, verify that the rail will not come into collision with any other objects when it homes.
Then press and hold the green button (see below).
Once the green light is turned on, the rail should start homing, and you can release the button.

![Alt text](media/RailSafetyButton.jpg){width=50%}

## How it works

An arduino is running inside of the box which is plugged into the computer through a usb cable and communicates through a serial interface.
This arduino is set up through udev rules to have a symlink to `/dev/safety_com_port`, which is the serial interface that the [vention rail hardware interface](vention_rail_hardware_interface/src/vention_rail_hardware_interface.cpp) is looking for.

If the hardware interface finds the arduino, the hardware interface sends a message, `poll`, to the arduino.
The arduino checks every 250ms for a poll, and if it receives a poll, it will check to see if the button is pressed.
If the button is pressed, the arduino will send the value `113`, and light up the green button.
If it is not pressed, it will return `86` (number chosen so that it would require more than one bit flip to be wrong).
The hardware interface checks whether the value returned was 113, and if it was, it moves on with the remainder of the rail configuration, including the homing.

The arduino code is available in [Homing_Button.ino](./arduino/Homing_Button.ino).

## Citation

This project falls under the purview of the iMETRO project. If you use this in your own work, please cite the following paper:

```bibtex
@INPROCEEDINGS{imetro-facility-2025,
  author={Dunkelberger, Nathan and Sheetz, Emily and Rainen, Connor and Graf, Jodi and Hart, Nikki and Zemler, Emma and Azimi, Shaun},
  booktitle={2025 22nd International Conference on Ubiquitous Robots (UR)},
  title={Design of the iMETRO Facility: A Platform for Intravehicular Space Robotics Research},
  year={2025},
  volume={},
  number={},
  pages={390-397},
  keywords={NASA;Moon;Seals;Maintenance engineering;Maintenance;Robots;Standards;Open source software;Testing;Logistics},
  doi={10.1109/UR65550.2025.11077983}}
```
