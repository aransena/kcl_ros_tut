# What is ROS?
ROS stands for 'Robot Opertaing System', but it is not an 'operating system' in the way you would be used to - ROS is a collection of software packages that are installed like any other software. ROS offers software to allow for easy communication between a distributed set of programs, whether they're locally distributed as a number of different programs running on your computer, or a collection of programs running over a network.

The 'Operating System' part of the ROS name refers to the idea that a robot system is often not just any one computer controlling everything, but a collection of subsystems that work together to achieve the top-level robot behaviour. ROS forms the 'glue' that joins it all together.

In addition to allowing for communication in a distributed system, ROS provides a rich library of packages that are commonly used in robot development. Everything from packages that easily pull in depth sensor data, to connecting your robot to web-applications, to a pre-built customisable navigation system for mobile robots.

The notes here are to give you my perspective on the key things you need to learn to be comfortable in ROS to the point that you'll be able to get up and running with new packages and ROS concepts smoothly, rather than being completely comprehensive. http://wiki.ros.org/ is your best go-to for information on packages/tutorials etc. as well as the usual like stack overflow.

# Pre-requisites
- A computer running Ubuntu 14.04
OR
- A Windows machine running Ubuntu 14.04 in a virtual machine.

## Recommended:
### Terminator
This is a very useful terminal emulator which allows for multiple terminals in one window. ROS is largely run from a terminal window, so it's not uncommon to have 6+ terminals open at one time - Terminator greatly simplifies managing them all. 

To install Terminator, the easiest option is to open the Ubuntu Software Center and search for Terminator, it should be the first result.

### Nootrix Image
If you're using a virtual machine, a useful resource is the VM image from Nootrix which has ROS Indigo pre-installed on Ubuntu 14.04
Info - http://nootrix.com/software/ros-indigo-virtual-machine/
Download link - https://drive.google.com/file/d/0B_ULD8CBaw26TEpqUG1McGs5aVU/view?usp=sharing


# Installing ROS
There are multiple versions of ROS available (http://wiki.ros.org/Distributions), but the version we'll be using is ROS Indigo. Distributions are not necessarily cross-compatible, so it's important to stick to one version for a project.

Rather than write out the installation process out in full here, head over to the ROS wiki for the installation procedure: http://wiki.ros.org/indigo/Installation/Ubuntu

The ROS wiki will become your best friend - all released packages will have an info pack on this wiki, and many packages have some tutorials to help get you started.

# Getting Started
Before you go about creating our own code, let's take some time to get familiar with the main concepts.

## ROS Core
As the name might suggest, the ROS Core is the central part of a ROS system. In order to start running anything with ROS, you must first launch roscore.

In a terminal, simply enter:

```
roscore
```

There's quite a lot to roscore, but for the most part you won't have to worry about it for now. Just remember to always run roscore before you try run anything else ROS related.

## Packages
With ROS, software is organised into *packages*. One of the main benefits of ROS and it's open-source nature is that there are many packages pre-built for you to use that provide a huge amount of functionality out of the box.

For the purpose of learning about ROS, a common starting point is the turtlesim package. To run code from this package, we use the ROS command rosrun.

```
rosrun turtlesim turtlesim_node
```
You should see a window like this pop up:

![turtlesim-node](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtlesim.png)

Packages can contain multiple, related, runnable files. If you can't remember the exact name of the file you want to run, but can remember or guess the package it's in you can always use Tab completion to check:
```
rosrun tu[TAB] 
rosrun turtles[TAB}
rosrun turtlesim [TAB][TAB]
--> outputs: draw_square        mimic              turtlesim_node     turtle_teleop_key
rosrun turtlesim turtles[TAB]
rosrun turtlesim turtlesim_node[ENTER]
```

If you want to install a new package, you use:
```
sudo apt-get install ros-indigo-<Package Name>
```

## Nodes & Topics
Main tutorial: http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics

When you launch the turtlesim window you are setting up a ROS *Node*. Each package might have a few different Nodes that it can launch. ROS Nodes communicate with each other via *topics*. The turtlesim package has another node for teleoperation, turtle_teleop_key - run it and see how you can control the turtle character.

You can visualise how these two nodes are connected using the *rqt_graph* tool
```
rqt_graph
```

You should see something like this:
![turtlesim-teleop](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png)

The ovals are the nodes, the arrow is the topic labelled with the name of the topic. The topic arrow shows the directional relationship between the nodes, with the teleop node controlling the turtlesim node. We refer to the teleop node as a *publisher* and the turtlesim node as a *subscriber*. In this case, each node is either a publisher or a subscriber; however a node can act as both a publisher and a subscriber (i.e. read data, process it, publish values on another topic).

A command line tool for inspecting topics is *rostopic*. rostopic has a number uses, accessed through arguments:
```
rostopic [TAB] [TAB]
outputs: bw    echo  find  hz    info  list  pub   type
```
http://wiki.ros.org/rostopic

- bw: rostopic bw /[topic name] - evaluates the bandwidth useage of a topic
- echo: rostopic echo /[topic name] - displays all messages being published to the topic
- find: rostopic find [message type] - displays all topics publishing messages of the given type
- hz: rostopic hz /[topic name] - displays update frequency of a published topic
- info: rostopic info /[topic name] - displays topic message type, publishers, and subscribers.
- list: rostopic list - displays all currently active topics
- pub: rostopic pub /[topic name] [message type] [message] - allows you to publish messages to a topic directly from command line
- type: rostopic type /[topic name] - tells you the message type for a topic

Of these, you will probably find yourself using list, info, echo, and pub most frequently.

There is a similar tool for node information - *rosnode*. Investigate what arguments you can use with rosnode.
http://wiki.ros.org/rosnode

### Message Types
ROS has a number of pre-built message types that can be used to communicate between nodes. You can see what message types there are by investigating the std_msgs/msg folder

```
roscd std_msgs/msg
ls
```

We'll see later how to create custom message types that allow you to incorporate meta data directly with a message for ease of documentation/collaboration/etc as part of best practices.

## Data Visualisation & Logging Tools
### rqt_plot
A useful tool for visualising topic data in a more intuitive way than rostopic echo is *rqt_plot*. Try run rqt_plot now 
rqt_console. In the topic entry box, enter / and then select the pose topic. Then click on the remove icon and remove all but the linear velocity item.

Now as you teleop the turtlesim around you will see the graph update.

### RViz
With more advanced systems, a more powerful visualisation tool is rviz. There isn't much to see using this tool with turtlesim, but if you follow these two tutorials you should be able to see how useful it can be:

- How to launch a simulation of a turtlebot using the Gazebo simulator: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Gazebo%20Bringup%20Guide
- Teleoperate the turtlebot and visualise its sensors: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Explore%20the%20Gazebo%20world

Note that Gazebo can be pretty resource intensive.

### rqt_console
http://wiki.ros.org/rqt_console
ROS incorporates a debugging message system with different levels of messages. Using rqt_console, you can filter messages by severity level and monitor what is happening in your system. You can also issue your own ROS system messages and monitor them through rqt_console.


## Services & Parameters
Main tutorial: http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams

Publishers and subscribers constantly communicate back and forth to stream data; however sometimes it is useful to be able to be able to call on certain functionality only when it is needed, which is where services are useful.

e.g. filtering a point cloud, which requires quite a lot of processing power, and might be useful to a few different robot subsystems, so we put the filtering into a service that can be called upon when needed where the subsystems provide the data that requires filtering, and the service returns the filtered data.

This can also be used to offload heavy processing to another computer that is part of your ROS network, if you are operating in a distributed setup (something to discuss another day http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

## Recording Data
Main tutorial: http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data

When dealing with dozens of topics, it can be a bit inconvenient trying to write your own code that captures all the topics and data you would want; fortunately ROS provides the rosbag tool for data recording purposes, as well as data playback.

Say we wanted to record how a user interacted with our turtlesim node, we can run:

```
rosbag record /turtle1/cmd_vel
```

This will start recording all data on the /turtle1/cmd_vel topic. Move your turtle around for a while with a teleop node, then return to the rosbag window and press ctrl+C to end the recording session.

Your rosbag file (a .bag file) should be stored in the folder you ran rosbag from. You can get an overview of the contects of the file with 

```
rosbage info <name of file>.bag
```

Restart your turtlesim node, and then run 

```
rosbag play [name of your bag file].bag
```

You should see your turtle moving around as ROS plays back the cmd_vel messages.

If you want to record multiple topics, you can use the -O subset modifier and then list the topics you are interested in as arguments:

```
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

If you are interested in recording all topics, simply run:

```
rosbag record -a
```


# Creating your Own Packages

We'll go through how to get started with creating your own package; however when it comes to actually writing publishers/subscribers etc., it's best to head over to the main tutorial pages (links below).

An important thing to note when you are creating your own publishers/subscribers/etc. in the tutorials linked below is to note the changes required in your package's package.xml and CMakeLists.txt files.

Another important thing to note is that you will also find a CMakeLists.txt file in the location ~/catkin_ws/CMakeLists.txt; however this is NOT the one you edit. You edit the file located in ~/catkin_ws/src/my_pkg/CMakeLists.txt.

## Create a ROS Workspace
Main tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

As part of the installtion procedure, you will have set up your terminal environment to use ROS commands, but you will need set up a *workspace* for your development code.
```
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
```
Note that here, we have named our workspace *catkin_ws*, but we could have named it anything we wanted. A convention is to use lowercase letters and to add *_ws* to the name.

Your folder structure will now be like this:
```
catkin_ws/
  src/
```
We now need to *make* the workspace to finalise setup:
```
catkin_make
```
This will give us the folder structure:
```
catkin_ws/
  src/
  build/
  devel/
```
Before you can run code that you have developed in your workspace, you need to *source* your workspace:
```
source devel/setup.bash
```
If you forget this part after making your workspace, commands like *rosrun* won't be able to see any of the packages you develop in the workspace.

One tip if you ever have issues/errors running catkin_make, *before* digging through code or config files to try find an error, is to delete your build and devel folders, and then try catkin_make again from scratch. You can use the command rm -rf [folder name/folder path] to delete a folder.

## Creating Your Own Packages
Main tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingPackage
Packages have a specific structure, requiring both a *package.xml* file which provides meta information on the package, and a *CMakeLists.txt* file which is used by the catkin build system.

Fortunately a lot of the legwork is done for us using the catkin package tool. Let's create our own package, and build up some code to control the turtlesim node.
```
cd ~/catkin_ws/src
catkin_create_pkg my_pkg rospy std_msgs
```
This command will create a new folder in our src folder called *my_pkg*. In my_pkg, you will find the auto-generated package.xml and CMakeLists.txt files. 

In these files, dependencies will already be configured for rospy and std_msgs. rospy is the python ros library we'll need to use in our code, and std_msgs are the standard ROS messages we'll need to communicated with the turtlesim node.

```
catkin_create_pkg [Package Name] [Dependency 1] [Dependency 2, etc.]
```

It's often useful to introduce a folder structure to your package, for example we could have:
```
cd example_pkg
mkdir src <-- For the main code files
mkdir msg <-- For custom message types
mkdir srv <-- For service description files
mkdir launch <-- For ROS launch files
mkdir res <-- For package resources, e.g. images, sounds
mkdir urdf <-- For robot universal robot description format files, used in simulators like gazebo
```
Of course, you don't need all of them.

## Publishers & Subscribers
Python: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
C++: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

Note before you can run a python file from your package, you will need to set permissions for it:

```
sudo chmod a+x <path to your file.py>
```

## Custom Message Types and Declaring Services
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

## Services
python: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
c++: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

## Running your package
Remember to catkin_make your workspace and then source it before trying to rosrun your package.

## Exercise
Once your done with the ROS tutorials, create a package that can control the turtlesim and make it drive in circles.

## Pulling in Packages from Git
ROS workspaces use wstool to manage code from external sources. Say you wanted to clone this repo (which is set up like a package, with a package.xml file and CMakeLists.txt file), you would enter the following commands:

```
cd ~/catkin_ws/src
wstool init
wstool set kcl_ros_tut --git https://github.com/aransena/kcl_ros_tut
wstool update
```

The format for the wstool set command is:

```
wstool set [local name] [source] [link]
```

# ROS Resources

## Main Tutorials Index
http://wiki.ros.org/ROS/Tutorials

## Cheatsheet
You'll find a really useful cheatsheet here: https://github.com/ros/cheatsheet/releases/download/0.0.1/ROScheatsheet_catkin.pdf

You wind up using pretty much all of these commands quite regularly, so worthwhile getting familiar with them!

## Useful Packages
Some very useful packages that you may want to investigate further once you've gotten to grips with ROS are:
- SMACH, Python library for building hierarchical state machines: http://wiki.ros.org/smach
- ROSBridge, ROS tool for connecting a ROS system to a non-ROS system (e.g. smart watch/Windows Machine/server/website/etc.): http://wiki.ros.org/rosbridge_suite
- usb_cam, ROS tool for reading in images from a usb camera and publishing them to a ROS topic: http://wiki.ros.org/usb_cam
- web_video_server, ROS tool for streaming images/video over a network: http://wiki.ros.org/web_video_server
- kinect_aux, package that lets you control the Motor and LED in a Kinect: http://wiki.ros.org/kinect_aux
- joy, package for interfacing with any linux supported joysticks (e.g. xbox controller): http://wiki.ros.org/joy
- openni2_launch, package for interfacing with an Xtion Pro Depth Sensor (but not a Kinect): http://wiki.ros.org/openni2_launch
- openni_launch, package for interfacing with a Kinect v1: http://wiki.ros.org/openni_launch
- To interface with a Kinect v2, see here: http://www.ros.org/news/2014/09/microsoft-kinect-v2-driver-released.html

## Important Tools
Some important tools specific to working in RLL
- MoveIt, Motion Planning Framework: http://moveit.ros.org/

