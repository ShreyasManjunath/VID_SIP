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
