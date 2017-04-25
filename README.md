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


# Getting Started with ROS
There are multiple versions of ROS available (http://wiki.ros.org/Distributions), but the version we'll be using is ROS Indigo. Distributions are not necessarily cross-compatible, so it's important to stick to one version for a project.

Rather than write out the installation process out in full here, head over to the ROS wiki for the installation procedure: http://wiki.ros.org/indigo/Installation/Ubuntu

The ROS wiki will become your best friend - all released packages will have an info pack on this wiki, and many packages have some tutorials to help get you started.
