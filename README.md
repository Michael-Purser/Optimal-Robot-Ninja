# Optimal robot ninja

<!---
***NOTE:  
This repository is not being updated for the moment, as C++ implementation work is being done in a template defined in the GIP 2018 repository and will be added here later. A link to this repo can be found [here](https://github.com/intermodalics-gip/omg_ros).***
-->

**NOTE: C++ implementation work is currently being done in a branch of the GIP 2018 repository containing the code template. The current repo is mostly for testing purposes, although at some point in the future the code will be added here too. A link to the GIP repo can be found [here](https://github.com/intermodalics-gip/omg_ros).**

##### Table of contents 
[About](#about)
[Contents](#contents)
[Robot platform](#platform)
[C++ Implementation log](#implementation_log)

<a name="about"\>
## About
This repository contains the matlab and C++ code written for the master thesis 'Optimal robot ninja: making robots move in an optimal and responsive way'. The goal of this thesis is to make a mobile robot platform move optimally in an environment containing static (optionally also dynamic) obstacles. Optimality is defined with respect to time. The project is a collaboration between [Intermodalics](https://www.intermodalics.eu/) and the [MECO research group](https://www.mech.kuleuven.be/en/pma/research/meco/) at KU Leuven.

<a name="contents"\>
## Contents
The repository contains following folders:
 - __/matlab__: matlab code and simulation environment.
 - __/python__: python implementation of matlab code, with an architecture that fits the ROS environment used at Intermodalics (*abandoned, switched to C++*)
 - __/c++__: C++ implementation of the code

The current state of the code architecture can be found [here](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/architecture.png) (click on link, then right click on figure and open in new tab to see full figure).

<a name="platform"\>
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




<!---
## Current work
Current working points, in order of importance:
 - Adaptation of code architecture to fit within the ROS-framework used at Intermodalics. Addition of thread separation. (update: after discussion, decided to work in [this repo](https://github.com/intermodalics-gip/omg_ros) for now; later code will be added to this repo again).
 - Implementation of conversion costmap -> point measurements.
 - Examination of new functionality taking into account initial environment knowledge.
 - Examination of new functionality dealing with the delay between measurement and actuation (due to processing and planning).
 - Examination of pointers and memory preallocation to make the C++ code run faster.
-->

<a name="implementation_log"\>
## Implementation log

Log of **C++ implementation** work.  

#### Completed
ROS/gazebo/rviz workflow:
 * Run turtlebot simulation on ROS kinetic (see [turtlebot tutorials](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation))
    * read out some sensor topics using `rostopic echo`
 * Visualise turtlebot info using rviz (see [turtlebot rviz tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/3D%20Visualisation)).
 * Create and build ROS package 'my\_first\_package'
    * implemented a talker, a listener for '\\joint\_states' topic and a listener for '\\camera\_depth\_points' topic
    *Note 1: turtlebot simulation should be running for listeners to work*
    *Note 2: turtlebot does not use std\_msgs but sensor\_msgs*
    * print some info to the command line 
    * check that the created nodes insert correctly in the graph using `rosrun rqt\_graph rqt\_graph`
 * Follow [localization and navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) tutorials
    * *Note 1: teleop node necessary to perform localization, but must be shut down before navigation is called
    Otherwise robot will not move because velocity commands from teleop node take precedence over local\_planner node* 
    * *Suggestion: remove need for localization by hardcoding robot initial position*


#### In progress
 * Write the local planner, fit it into class provided by Nikolaos.
    * Writing a print that shows up in console when simulation is launched
    * Sending a stright ahead velocity command to the robot

#### To do
 * Implement code in C++:
    * Blocks 'getNextLocalGoal', 'getConstraintValues' and 'getInitialGuesses' (see [code architecture ](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/architecture.png))
 * Write chapter 3 of thesis text

