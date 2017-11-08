# beginner_tutorials

## Overview 

This tutorial demonstrates how publisher and subscriber nodes work to communicate across a ROS network. A node is an executable that uses ROS to communicate with other nodes on the network. These nodes can publish or subscribe to topics to send and receive messages to each other.

This tutorial also demonstrates how a ROS service works to expect a message and return a message when it is called. The Talker node in this package communicates with the StringFlip service which alters the message that is being published by reversing the string that gets sent as a request. 

## Dependencies

 - ROS Kinetic is installed on machine
 - "beginner_tutorials" is cloned in a configured catkin workspace

## Build

 - Clone repo (https://github.com/sjohns09/beginner_tutorials.git) to catkin workspace
 - Switch to Week10_HW branch
 - In catkin workspace root directory, run catkin_make. This will build the "beginner_tutorials" project.
 
 ```
 cd ~/catkin_ws/src
 git clone https://github.com/sjohns09/beginner_tutorials.git
 cd beginner_tutorials
 git checkout Week10_HW
 cd ~/catkin_ws
 catkin_make
 ```

## Run (Each Node)

 - Start up roscore in its own terminal ($ roscore)  
  - In a new termianl, run the service node ($ rosrun beginner\_tutorials string\_flip)
 - In a new termianl, run the publisher node ($ rosrun beginner\_tutorials talker)
 - In a new terminal, run the subscriber node ($ rosrun beginner\_tutorials listener)
 
** In each new terminal source the setup file ($ source ./devel/setup.bash)

## Run (Launch File)

 - In a new terminal source the setup file ($ source ./devel/setup.bash)
 - Call the week10hw.launch launch file which requires a command line argument setting the loop\_rate. This determines how quickly the talker will publish messages 
 
The example below will launch the string\_flip, talker, and listener nodes. This will begin publishing messages and calling the string\_flip service. Log messages for the talker and listener will be displayed in their own console windows at 1 Hz. (Any loop\_rate value can be entered)

```
roslaunch beginner_tutorials week10hw.launch loop_rate:=1
```

## Call Service From Command Line

To run the service from the command line, first start the string\_flip node, and then in another terminal call the service string\_flip as demonstrated below. (Any string can be taken as input)

```
rosrun beginner_tutorials string_flip
rosservice call /string_flip "Flip This!"
```
__output: !sihT pilF__

## License

BSD 3-Clause (See LICENSE file for details)
