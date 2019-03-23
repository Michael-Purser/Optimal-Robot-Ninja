# Optimal robot ninja

This repository contains the matlab and python code written for the master thesis 'Optimal robot ninja: making robots move in an optimal and responsive way'. The goal of this thesis is to make a mobile robot platform move optimally in an environment containing static (optionally also dynamic) obstacles. Optimality is defined her with respect to time. The project is a collaboration between Intermodalics and the MECO research group at KU Leuven.

## Structure of this repo
The repository contains following folders:
 - __/matlab__: matlab code and simulation environment.
 - __/python__: python implementation of matlab code, with an architecture that fits the ROS environment used at Intermodalics, more specifically the move-base-flex package.
 - __/c++__: C++ implementation of the code

The current state of the code architecture can be found [here](https://drive.google.com/open?id=1vqejaQd0AvWK3YYID0Eymey2rQr1e2JI).

## Robot platform
The mobile platform used for implementation is a [Turtlebot 2](http://www.willowgarage.com/turtlebot), together with a custom vision system developed at Intermodalics.  
Platform characteristics (full datasheet available for download [here](http://bit.ly/1L2FIzG)):
 - diameter: 354 mm, height: 420 mm
 - weight: 6,3 kg (bottom-heavy), max payload: 5 kg
 - max speed: 0,65 m/s, max rotational speed: 180°/s
 - vision system: depth measurement (dual camera)
 - noise on measurements: ~ 3 cm
 - costmap availability frequency: 5 Hz
 - localization availability frequency: 20 to 25 Hz
 - localization accuracy: a few cm

![figure of turtlebot](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/turtlebot2_info.png "Turtlebot 2 schematics")

## Current work
Current working points, in order of importance:
 - Adaptation of code architecture to fit within the ROS-framework used at Intermodalics. Addition of thread separation.
 - Implementation of conversion costmap --> point measurements.
 - Examination of new functionality taking into account initial environment knowledge.
 - Examination of new functionality dealing with the delay between measurement and actuation (due to processing and planning).
 - Examination of pointers and memory preallocation to make the C++ code run faster.

