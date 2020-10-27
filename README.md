# The Motion Manager for generating upper-limb motion in humanoid robots 
This application is designed and developed as a ROS package for the implementation, monitoring and application of novel planning, learning and controlling techniques for humanoid robots. By the ROS networking capabilities, the Motion Manager communicates with simulated and real robotic devices for processing sensorial information and tune the algorithms under investigation. The developed Graphical User Interface (GUI) has been changing over the versions for a more intuitive usage and a clearer analysis of the monitored variables. It has been tested on ROS indigo, the simulator V-REP EDU v 3.4.0 and the Qt4.8 libraries.    

Currently supported robots:
* the **A**nthropomorphic **Ro**botic **S**ystem **ARoS**: see its [URDF model](https://github.com/zohannn/aros_description)

## Overview
The purpose of this document is to provide a brief overview of the main features, while more technical details are described in the [Wiki pages](https://github.com/zohannn/motion_manager/wiki). The Motion Manager GUI is composed by four tabs with independent functionalities, namely: **Scenario**, **Planning**, **Learning**, **Controlling** and **Results**.    

### Scenario

A V-REP simulated scenario|
------------- |
![V-REP](/resources/images/vrep.png)|

Simulated [scenarios and models](https://github.com/zohannn/scenarios) of the anthropomorphic robot **ARoS** are developed and available for the [RViz visualizer](http://wiki.ros.org/rviz) and the simulator [V-REP](https://www.coppeliarobotics.com/) 

The Scenario tab | 
------------ | 
![Tab Scenario](/resources/images/scenario.png) | 

After a first check of a communication with ROS and the simulator, the positional, orientational and dimensional information of the objects populating these scenarios can be read by the Motion Manager, which is also acknowledged of the initial configuration of the robot.

### Planning
The Planning tab |
-----------------|
![Tab Planning](/resources/images/plan.png) |

The **Planning** tab is an user interface that allows the graphical settings of goal-directed movements and of the parameters of a specified trajectory planner. The latter is selected among a group of algorithms for path planning, which are provided by the [MoveIt! libraries](https://moveit.ros.org/) and equipped with the different time parametrizations. For instance, the planner [HUMP](https://github.com/zohannn/HUMP) for human-like motion generation can always be selected, while the planners

- RRT
- RRTConnect
- PRM
- RRT*
- PRM*

are also at disposal of the user when the [MoveIt!-based ARoS planning library](https://github.com/zohannn/aros_moveit_planner) is linked to the Motion Manager.
A statistical collections of data concerning the bi-dimensional and three-dimensional power laws applied for the robotics domain are possible through this tab. Moreover, a planned movement can be sent for execution on the V-REP simulator, be saved and added to compose a task that sequentially accomplishes different goals in the given scenario.   

### Learning
The Learning tab |
-----------------|
![Tab Learning](/resources/images/learn.png) |

The **Learning** tab allows to set positional and orientational variations that may randomly occur in similar, but never equal, scenarios. In such situations, supervised machine learning techniques can be applied to train the capability of predicting good solutions and of feeding a movement planner with meaningful information. In this tab, the planner [HUMP](https://github.com/zohannn/HUMP) can be started several times with a default initialization to collect the so-called "cold-started data". The "warm-started data", on the contrary, are collected by iteratively running the planner [HUMP](https://github.com/zohannn/HUMP) with previously collected solutions in slightly different scenarios. Based on a Python class for posture learning (see [HUPL](https://github.com/zohannn/HUPL)), the Motion Manager can use these databases to call user-selected Python scripts and apply training, predicting and forgetting policies for a meaningful initialization of a motion planner.       

### Controlling
The Controlling tab |
--------------------|
![Tab Controlling](/resources/images/control.png) |

The **Controlling** tab is an interface that permits the setting of a second-order closed-loop controller, which is designed and developed for the operational-space tracking of given human-like upper-limb trajectories. The redundancy of a 7-DOFs manipulator is used for the on-line avoidance of physical joint limits, singularities of the Jacobian matrix and of obstacles in the scenario. The reference level of human-likeness is maintained by tracking planned hand and elbow angle trajectories, while time is dynamically mapped with respect to the start-goal distance in the operational space of manipulation. Noise in the acquisition of the kinematic variables of interest can be here mitigated by low-pass filtering the data incoming from the real robot and by robustly calculating higher derivatives. The on-line execution of a movement can be safely performed on either the simulated or the real robot.          

### Results
The Results tab |
--------------------|
![Tab Results Hand](/resources/images/results_hand.png) |
![Tab Results Joints](/resources/images/results_joints.png) |
![Tab Results Track](/resources/images/results_track.png) |

The **Results** tab collects and processes the data that emerge from the plannig, the learning and the controlling stages. These data can be visualized, compared and saved to perform furhter analysis.  


## Upgrades
* The Motion Manager has been upgraded by [Sara Sá](https://github.com/sarasa22) to support the collaborative robot Sawyer by Rethink Robotics&copy; (see [https://github.com/sarasa22/motion_manager](https://github.com/sarasa22/motion_manager))  
* The Motion Manager has been upgraded by [JoaoQPereira](https://github.com/JoaoQPereira) to support the collaborative robot UR5 by Universal Robots&copy; (see [https://github.com/JoaoQPereira/motion_manager](https://github.com/JoaoQPereira/motion_manager)) 

