# Optimal Robot Ninja: work progress

Online log of performed **C++ implementation** work.  

## Completed
Determine ROS/gazebo/rviz workflow:
 * Got the [turtlebot tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) working in Gazebo on ROS kinetic
 * Visualise turtlebot info using rviz (see [turtlebot rviz tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/3D%20Visualisation)).
 * Read out some of the topics of the above gazebo turtlebot simulation with `rostopic echo` command.
 * Created ROS package 'my\_first\_package'
    * has three nodes: a talker, a listener for the 'joint\_states' topic and a listener for the depth camera image topic 'camera\_depth\_points'
    * prints some info to the command line 
    * checked that the created nodes insert correctly in the node/topic graph using `rosrun rqt\_graph rqt\_graph`
    * note 1: gazebo turtlebot simulation should be running for these listeners to work ([here](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) is the tutorial I adapted)
    * note 2: there are not only std\_msgs in ROS but also other types of messages such as sensor\_msgs, which is what the turtlebot uses.
 * Followed the [localization and navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) tutorials. 
    * note: teleop node is necessary to have correct localization, but must be shut down before the navigation is called! Otherwise the robot doesn't move as velocity commands from the teleop node take precedence over the DWA node. *Suggestion: remove need for localization by hardcoding robot initial position*.
    * DWA planner performs very poorly when going through tight spaces, 'dancing' around (spinning on it's axis before finally recovering).


## In progress
 * Writing the local planner class, fitting it into structure provided by Nikolaos.
    * Writing a print that shows up in console when simulation is launched
    * Sending a stright ahead velocity command to the robot

## To do
 * Implement code in C++:
    * Blocks 'getNextLocalGoal', 'getConstraintValues' and 'getInitialGuesses' (see [code architecture ](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/architecture.png))
 * Write chapter 3 of thesis text
