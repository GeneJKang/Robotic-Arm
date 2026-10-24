# Robotic Arm (Page WIP)

A 6-axis robotic arm built in ROS 2 with closed-form inverse kinematics, and closed-loop stepper motor build controlled over Wi-Fi

<img width="800" height="875" alt="ScreenRecording2026-10-20214917-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/9b649941-9c52-46f9-ae92-e577bbb665f7" />

## Highlights

+  Closed-form 6-DOF Inverse Kinematics
    + Uses kinematic decoupling at a spherical wrist, solving the position first and then the orientation.
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

<img width="1678" height="789" alt="image" src="https://github.com/user-attachments/assets/3ea1f413-6942-44d6-9e12-2f46bace1204" />
<sub><i>Diagram designed in Miro</i></sub>

Diagram will be updated as this project progresses.

## Kinematics

| Joint | Axis | Function | Limit (rad) |
| --- | --- | --- | --- |
| J1 | Z | Base yaw | ±π |
| J2 | Y | Shoulder pitch | ±π/2 |
| J3 | Y | Elbow pitch | ±2.6 |
| J4 | Z | Wrist roll | ±π |
| J5 | Y | Wrist pitch | ±2.1 |
| J6 | Z | Tool roll | ±π |

## Forward Kinematics

Forward kinematics chains 4x4 homogeneous transforms together. It has one per joint plus a fixed tool offset.

## Inverse Kinematics

The inverse kinematics works in four steps:
1. Back the wrist center off from the target along the tool's Z axis.
2. Find J1 from the wrist center's direction in the XY plane, and as consequence, reduces J2 and J3 to a planar two-link problem.
3. Solve J2 and J3 with law of cosines.
4. Compute the remaining wrist rotation R<sub>3 6</sub> = R<sub>0 3</sub><sup>T</sup> * R_target, and read J4-J6 off it as ZYZ Euler angles.

The solver returns the first configuration that satisfies every joint limit, or None if the pose can't be reached.

## Network and Discovery

Normal ROS 2 discovery uses multicast, which Wi-Fi access points often delay, drop, or block which is an issue. A Fast DDS Discovery Server on the Pi replaces it with unicast where each node registers with the server and gets back the addresses of its peers. It is a better alternative to what I am trying to do and once the nodes have found each other, topic data flows directly between them and never passes through the server.

The FastDDS discovery server runs on the Pi which is run by the fastdds-discovery.service. I had issues with WSL rejecting the inbound traffic but this shouldn't be an issue for regular Linux users. I ended up using a Hyper-V firewall and changed how my IPs were set up for all WLAN connections on my WSL Ubuntu instance.

Here are the settings I used for the discovery server:

[Unit]

Description=Fast DDS Discovery Server for ROS 2

Wants=network-online.target

After=network-online.target


[Service]

Type=simple

User=CHANGE_ME

ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && exec fastdds discovery -i 0 -p 11811'

Restart=on-failure

RestartSec=2


[Install]

WantedBy=multi-user.target


## Getting Started

git clone https://github.com/GeneJKang/Robotic-Arm.git

cd Robotic-Arm

rosdep install --from-paths src --ignore-src -y

sudo apt install python3-scipy

colcon build --symlink-install

source install/setup.bash

ros2 launch arm_description display.launch.py

RViz should launch and you can just drag the target marker (InteractiveMarker) and the arm will track it.

## Hardware

| Component | Part |
| --- | --- |
| Shoulder joints | 57HB250-112B hybrid servo (NEMA 23 closed-loop) + HB808C Motor Driver |
| Elbow / Wrist joints | NEMA 17 steppers + TMC2209 Motor Drivers |
| Controller | Raspberry Pi 5 (networking) + Arduinos (Step Generators) |
| Power | Mean Well LRS-350-24 (24 V, 350 W) with buck converters |

The encoders on the motors are incremental so the position is lost after every power cycle. You could just never turn off the robot but the best thing to do is to have homing switches and at startup just zero in your motors.

The reason why I moved the step generators from the Pi to the Arduinos is due to jitter I experienced after testing. I ran the signals through a oscilloscope and with ROS running in the background the scheduler had a hard time keeping up with the pulses needed. I took inspiration from Klipper's 3D printers.

I decided that when the IK has no valid solution the arm keeps publishing the last good configuration. It could go without it and the program can handle that but I decided it was better off to get rid of any unpredictable actions.
