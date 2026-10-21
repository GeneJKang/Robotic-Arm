# Robotic Arm (Page WIP)

A 6-axis robotic arm built in ROS 2 with closed-form inverse kinematics, and closed-loop stepper motor build controlled over Wi-Fi

<img width="800" height="875" alt="ScreenRecording2026-10-20214917-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/9b649941-9c52-46f9-ae92-e577bbb665f7" />

## Highlights

+  Closed-form 6-DOF Inverse Kinematics
  - Uses kinematic decoupling at a spherical wrist, solving the position first and then the orientation.
  + The orientation uses ZYZ Euler angles and searches all 4 arm configurations during edge cases.
  + Handles gimbal-lock, utilizing quaternions for rotation, and rejects any solution outside the set joint limits.
+ Interactive Control in RViz.
  + The RViz setup has a InteractiveMarker publisher that you can add to drag and control the robot's end effector.
+ Hardware Design
  + Utilizes closed-loop NEMA 23, and NEMA 17 stepper motors with planetary gearboxes.
  + It will use contact switches to zero in the angles on the encoders.
  + Dedicated microcontrollers to generate step pulses to avoid scheduling jitter.
  + Uses a 24 V / 350 W AC to DC power supply

## System Architecture

<img width="2065" height="520" alt="image" src="https://github.com/user-attachments/assets/b5bcbb46-fd6f-4236-a3e7-8cd836b8b912" />
*Diagram designed in Miro*

Diagram will be updated as this project progresses.

## Kinematics

## Forward Kinematics

## Inverse Kinematics

## Getting Started

ros2 launch arm_description display.launch.py

## Hardware
