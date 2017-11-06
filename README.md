# beginner_tutorials

## Overview 

This tutorial demonstrates how publisher and subscriber nodes work to communicate across a ROS network. A node is an executable that uses ROS to communicate with other nodes on the network. These nodes can publish or subscribe to topics to send and receive messages to each other.

This tutorial also demonstrates how a ROS service works to expect a message and return a message when it is called. The Talker node in this package communicates with the StringFlip service to alter the message that it is publishing. 

## Dependencies

 - ROS Kinetic is installed on machine
 - "beginner_tutorials" is in a configured catkin workspace

## Build

 - Clone repo (https://github.com/sjohns09/beginner_tutorials.git) to catkin workspace using
 - In catkin workspace root directory, run catkin_make. This will build the "beginner__tutorials" project.
 
 ---
 cd ~/catkin_ws/src
 git clone https://github.com/sjohns09/beginner_tutorials.git
 cd ~/catkin_ws
 catkin_make
 ---

## Run

 - Start up roscore in its own terminal ($ roscore)  
  - In a new termianl, run the service node ($ rosrun beginner\_tutorials string\_flip)
 - In a new termianl, run the publisher node ($ rosrun beginner\_tutorials talker)
 - In a new terminal, run the subscriber node ($ rosrun beginner\_tutorials listener)
 
** In each new terminal source the setup file ($ source ./devel/setup.bash)

The listener will display the messages it hears from the topic, which should match up with what is being published by the talker.

## Call Service From Command Line

TBD

## License

BSD 3-Clause
