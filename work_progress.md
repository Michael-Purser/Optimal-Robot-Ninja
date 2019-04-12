# Optimal Robot Ninja: work progress

For the last two months of the thesis, I have to decided to keep an online log of performed **C++ implementation** work.  
For the sake of clarity, this log will not be by date - rather, as can be seen below, it splits the work into the 'completed', the 'in progress' and the 'to do' category.

## Completed
First, I had to get to grips with the ROS/gazebo/rviz interface and workflow; I therefore figured how to de the following things:
 * Get the [turtlebot tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) working in Gazebo on ROS kinetic, and steer the turtlebot around using keyboard teleop.
 * Visualise turtlebot info using rviz (following the [turtlebot rviz tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/3D%20Visualisation)).
 * Reviewed ROS concepts, and read out some of the topics of the above gazebo turtlebot simulation with a simple `rostopic echo` command.
 * Created a ROS package 'my\_first\_package'
    * can launch three nodes: a talker, a listener for the 'joint\_states' topic and a listener for the depth camera image topic 'camera\_depth\_points', and print some info to the command line. 
    * The gazebo turtlebot simulation should be running for these listeners to work! ([here](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) is the tutorial I adapted to do this). 
    * Learned that there are not only 'std\_msgs' in ROS but also other types of messages such as 'sensor\_msgs', which is what the turtlebot uses. 
    * Also checked that the created node inserts correctly in the node/topic graph (using `rosrun rqt\_graph rqt\_graph` command)
 * Followed the [localization and navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) tutorials. 
    * Note: the teleop node is necessary to have correct localization, but must be shut down before the navigation is called! Otherwise the robot doesn't move as velocity commands from the teleop node seem to take precedence over the DWA node. 
    * Also, the DWA planner performs very poorly when going through tight spaces, 'dancing' around (spinning on it's axis before finally recovering).


## In progress
 * Writing the local planner class, fitting it into structure provided by Nikolaos.
    * Writing a print that shows up in console when simulation is launched
    * Sending a stright ahead velocity command to the robot

## To do
 * Implement code in C++:
    * Blocks 'getNextLocalGoal', 'getConstraintValues' and 'getInitialGuesses' (see [code architecture ](https://github.com/Michael-Purser/Optimal-Robot-Ninja/blob/master/architecture.png))
 * Write chapter 3 of thesis text
