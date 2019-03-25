# Optimal robot ninja

***NOTE:  
This repository is not being updated for the moment, as C++ implementation work is being done in a template defined in the GIP 2018 repository and will be added here later. A link to this repo can be found [here](https://github.com/intermodalics-gip/omg_ros).***

This repository contains the matlab and python code written for the master thesis 'Optimal robot ninja: making robots move in an optimal and responsive way'. The goal of this thesis is to make a mobile robot platform move optimally in an environment containing static (optionally also dynamic) obstacles. Optimality is defined her with respect to time. The project is a collaboration between Intermodalics and the MECO research group at KU Leuven.

## Structure of this repo
The repository contains following folders:
 - __/matlab__: matlab code and simulation environment.
 - __/python__: python implementation of matlab code, with an architecture that fits the ROS environment used at Intermodalics, more specifically the move-base-flex package.
 - __/c++__: C++ implementation of the code

The current state of the code architecture can be found [here](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/architecture.png) (click on link, then right click on figure and open in new tab to see full figure).

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
 - Adaptation of code architecture to fit within the ROS-framework used at Intermodalics. Addition of thread separation. (update: after discussion, decided to work in [this repo](https://github.com/intermodalics-gip/omg_ros) for now; later code will be added to this repo again).
 - Implementation of conversion costmap --> point measurements.
 - Examination of new functionality taking into account initial environment knowledge.
 - Examination of new functionality dealing with the delay between measurement and actuation (due to processing and planning).
 - Examination of pointers and memory preallocation to make the C++ code run faster.

