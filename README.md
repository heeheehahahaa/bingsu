# Engineering Materials

This repository holds the engineering materials for a self-driving vehicle model competing in the WRO Future Engineers competition for the 2025 season.   

# Introduction
This document details the design and implementation of a robot built for a competition, utilising a custom 3D-printed chassis and a variety of off-the-shelf electronic components. The robot is designed using Autodesk Fusion 360 and manufactured with Fused Filament Fabrication (FFF) 3D printing using Polylactic Acid (PLA). It incorporates LEGO MINDSTORMS EV3 motors,  Benewake TFmini and TF luna LiDAR sensors, a JoinMax JMP-BE-2617 Compass, and an OpenMV H7 camera.

# Our solution
Design and Manufacturing

- Software: Autodesk Fusion 360 was used for both Computer-Aided Design and Computer-Aided Manufacturing
- Printing Technology: Fused Filament Fabrication (FFF) with Polylactic Acid (PLA) filament was chosen for its affordability, versatility, and suitability for structural components.

Key Components

- The compass mount is mounted as high as possible to prevent magnetic interference with other parts of the robot/disturbance from environmental factors
- Drive base structure attaches the front motors and wheels to other parts of the robot, including the back wheels and other mounts. This connects to the medium motor for driving the robot, the pivot points for the steering mechanism, and additional points to mount sensors at the front. The front and back parts are connected via a thin structure which is not possible to do with LEGO parts. This gives the robot the ability to steer at larger angles, reducing the turning radius and making the robot more agile. 
- The wheel mount attaches the front wheels to the base structure of the robot.
- There is an integrated 40-tooth equivalent spur gear which is driven by an 8-tooth gear attached to an EV3 Medium motor
- The sensor mount is placed at the back of the robot, attaching the back TF mini to the body of the robot.  We placed it such that the majority of the field-of-view (FOV) lies on the wall, even at the furthest distance.
- Motor mount attaches a medium motor downwards which controls the steering wheels. We decided to mount it lower to be certain that it is able to range against the 10cm tall walls.
- Camera mount attaches the camera to the front of the robot, below the front ToF. It is kept low, allowing the camera to detect the blocks in the obstacle challenge more easily. 

Microcontroller

- EVN Alpha Advantages: Compared to LEGO MINDSTORMS EV3 and NXT controllers, the EVN Alpha offers 64 holes on 5 sides for mounting, is more compact, and features a USB-C port for charging and programming. It also has 16 I2C channels, overcoming the 4-port constraint of other controllers.

Motors and Gearbox

- Motors: Two LEGO MINDSTORMS EV3 medium motors are used for their light weight and high speed. One motor drives the robot through a complex gearbox setup, and the other handles steering with high precision.
- Gearbox: The gearbox includes a combination of bevel gears and planetary gears to achieve a total reduction ratio of 14:5. This design optimises the robot’s speed and stability.
- Wheels: 62.4mm LEGO TECHNIC wheels provide high friction and stability, crucial for rapid changes in direction and precise movement.


Steering Mechanism
- We implemented Ackermann geometry to ensure accurate and slip-free turning by adjusting the speeds of the left and right wheels according to their respective radii during turns.


Power Source

- Battery: The robot uses two 18650 cells connected in series, providing 8.4V. The batteries are capable of lasting over an hour of continuous operation due to their high discharge rate and capacity.
- Regulation: On-board regulators supply 3.3V and 5V to various components, with ample current capability to support all sensors and peripherals.

Sensors

- TF Mini-S: Mounted at the front and back for distance measurement
- TF luna: Two sensors placed on the sides for accurate distance measurement with a range of up to 2 metres.
- JoinMax BE-2617 Compass: Provides accurate heading data and adjusts the robot’s movement to correct any deviation.
- OpenMV H7 Camera: Positioned at the front to detect objects such as traffic lights and blocks



Challenges

- Wall Tracking: The robot maintains a fixed distance from the wall using proportional control based on the current and target distances.
- Turns: The robot sets a fixed heading to navigate turns based on block positions, adjusting its movement as needed.

Software Functionality

- State Management: The robot operates in different states (Cases 0-8) to manage initial positioning, wall tracking, turning, and responding to specific blocks in the obstacle challenge.


