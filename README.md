VID Structural Inspection Planner (VID_SIP)
============================================
Author: *Shreyas Manjunath*

VID_SIP is a UAV path planning algorithm. It uses triangulated mesh (.stl) file as input and samples the viewpoints around the object and an optimized tour is generated using Traveling Salesman Problem (TSP) solver. It is a ROS-based implementation that provides the output as an usable Viewpoint list rostopic and a text file is also written out for using the tour file on different platforms.

Supports only Rotorcraft models.

VID_SIP is a code base developed as part of Master Thesis work of the author. 

Requirements
-------------
1. Ubuntu 18.04 (Tested on)
2. Robot Operating System (ROS) melodic 
3. Eigen3 package
4. CMake 3.14 or higher 
5. qpOASES [[1]](https://github.com/coin-or/qpOASES) or SIP/optec [[SIP]](https://github.com/ethz-asl/StructuralInspectionPlanner) 
6. Google OR tools

Installation
---------------
- Install Robot Operating System as per thier official documentation. [[ROS]](http://wiki.ros.org/ROS/Installation)
- Setup the ROS workspace using catkin_tools.
- Install Eigen3 library using the following command.
```
$ sudo apt install libeigen3-dev
```
-- For higher versions of Eigen3 library, refer to Eigen3 official website [[Eigen3]](https://eigen.tuxfamily.org/index.php?title=Main_Page) to install it from the source.

- Verify if CMake version is 3.14 or higher
- Install Google OR tools referring to the official installation page [[2]](https://developers.google.com/optimization/install). Use cmake build to generate CMake file for easy library usage.
- Clone this VID_SIP package into the ROS workspace src folder.
- Clone the qpOASES library for quadratic problem optimization, and add it into as a folder ’optec’ inside VID_SIP directory.
```
$ git clone https://github.com/coin-or/qpOASES
```
```
----VID_SIP
        |___ planner_node
        |___ request_client_node
        |___ *optec*
```
OR
- Get the library from the SIP/optec [[SIP]](https://github.com/ethz-asl/StructuralInspectionPlanner) folder as it is.
- Build the package using the following command.
```
$ cd ~/<Your ROS Workspace>/
$ catkin build planner_node
$ catkin build request_client_node
```

Usage
--------
1. Add your mesh inside request_client_node/meshes directory. The mesh file (.stl) should be strictly ASCII format.
2. A settings file available to set the parameters to the planner and request client. Use accordingly.
3. Launch file, planner_node.launch under planner_node/launch is used to launch planner_node, request_client_node and RVIZ visualization together. Use the following command to launch the node.
```
$ roslaunch planner_node planner_node.launch
```
