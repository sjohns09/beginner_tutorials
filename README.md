# beginner_tutorials

## Overview 

This tutorial demonstrates how publisher and subscriber nodes work to communicate across a ROS network. A node is an executable that uses ROS to communicate with other nodes on the network. These nodes can publish or subscribe to topics to send and receive messages to each other. 

## Dependencies

 - ROS Kinetic is installed on machine
 - "beginner_tutorials" is in a catkin workspace

## Build

In catkin workspace, run catkin_make. This will build the "beginner__tutorials" project.

## Run

 - Start up roscore in its own terminal ($ roscore)  
 - In another terminal go to the catkin workspace and source the setup.sh file ($ source ./devel/setup.bash)
 - In a new termianl, run the publisher node ($ rosrun beginner__tutorials talker)
 - In a new terminal, run the subscriber node ($ rosrun beginner__tutorials listener)

The listener will display the messages it hears from the topic, which should match up with what is being published by the talker.

## License

BSD 3-Clause
