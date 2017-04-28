Note: Don't clone this repo straight away - this readme will let you know when to clone!

# What is ROS?
ROS stands for 'Robot Opertaing System', but it is not an 'operating system' in the way you would be used to - ROS is a collection of software packages that are installed like any other software. ROS offers software to allow for easy communication between a distributed set of programs, whether they're locally distributed as a number of different programs running on your computer, or a collection of programs running over a network.

The 'Operating System' part of the ROS name refers to the idea that a robot system is often not just any one computer controlling everything, but a collection of subsystems that work together to achieve the top-level robot behaviour. ROS forms the 'glue' that joins it all together.

In addition to allowing for communication in a distributed system, ROS provides a rich library of packages that are commonly used in robot development. Everything from packages that easily pull in depth sensor data, to connecting your robot to web-applications, to a pre-built customisable navigation system for mobile robots.

# Pre-requisites
- A computer running Ubuntu 14.04
OR
- A Windows machine running Ubuntu 14.04 in a virtual machine.

## Recommended:
### Terminator
This is a very useful terminal emulator which allows for multiple terminals in one window. ROS is largely run from a terminal window, so it's not uncommon to have 6+ terminals open at one time - Terminator greatly simplifies managing them all. 

To install Terminator, the easiest option is to open the Ubuntu Software Center and search for Terminator, it should be the first result.

### Nootrix Image
If you're using a virtual machine, a useful resource is the VM image from Nootrix which has ROS Indigo pre-installed on Ubuntu 14.04 http://nootrix.com/software/ros-indigo-virtual-machine/


# Installing ROS
There are multiple versions of ROS available (http://wiki.ros.org/Distributions), but the version we'll be using is ROS Indigo. Distributions are not necessarily cross-compatible, so it's important to stick to one version for a project.

Rather than write out the installation process out in full here, head over to the ROS wiki for the installation procedure: http://wiki.ros.org/indigo/Installation/Ubuntu

The ROS wiki will become your best friend - all released packages will have an info pack on this wiki, and many packages have some tutorials to help get you started.

# Getting Started
Before you go about creating our own code, let's take some time to get familiar with the main concepts.

## Packages
What are packages
Finding packages
Running a package

```
rosrun tu[TAB] 
rosrun turtles[TAB}
rosrun turtlesim [TAB][TAB]
draw_square        mimic              turtlesim_node     turtle_teleop_key
rosrun turtlesim turtles[TAB]
rosrun turtlesim turtlesim_node[ENTER]
```
## Nodes & Topics
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
What are nodes/topics
Inspecting nodes/topics

rostopic [TAB] [TAB]
bw    echo  find  hz    info  list  pub   type

### Message Types
rostopic info 

## Logging Tools & Data Visualisation
rqt_plot
rqt_console


## Services & Parameters
http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams

# Create a ROS Workspace
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

# ROS Cheatsheet
You'll find a really useful cheatsheet here: file:///home/aransena/Downloads/ROScheatsheet_catkin.pdf

You wind up using pretty much all of these commands quite regularly, so worthwhile getting familiar with them!
