# Closed Loop Controller for the Tecdron: an omnidirectional mobile base with 4 wheels 

This repository includes the simulations made for synthesis and testing of the Tecdron, an omnidirectional mobile base with four mecanum wheels.
The main goal was developing a robust controller feasible to apply using the ros2_control framework. It includes the Matlab Simulations and the ROS2 environment for testing. 

![Tecdron_base](images/tecdron_base.png)


---
## Motivation

A holonomic base, like the Tecdron, can be highly useful in the industry due to its versatile movement capabilities.. Consequently, reliable and robust simulations of these types of robots are necessary. 
In parallel, ROS2 is increasing its presence in the industry, and the main simulator used for ROS2 is the Ign Gazebo (once the classic version is being descontinued).

However, as long as I could see, the simulations use an open loop in order to control the movement of the robot, using the inverse kinematics. 

![inverse_k](images/inverse_k.png)


Although this method can be really useful as an initial step and make the robot working e moving in gazebo, this is even the method taught in this open class of The Construct (https://www.youtube.com/watch?v=px4AJ5VE-1U&t=1234s), it's not very precise and the rotational movement in general has a very diferent behavior. So it could cause problems when you're testing SLAM and Navigations, for example, algorithms and consequently the results in the simulation probably will not be sufficiently reliable. 


That said, the main goal of this repository is to provide a simulated environment using ROS2 and Ign Gazebo with a robust controllers that we can obtain from the simulations in Matlab, in a way that the commanded velocity will match, as precisely as the control theory allows, the true velocity of the robot in the simulation. 


---
### System Especifications
- Ubuntu 22.04
- Matlab 2024 with robust control toolbox
- ROS Humble
- Ign Gazebo

### System Requirements
- Docker Engine
- NVIDIA Container Toolkit


## Usage

To build the image just command:

    docker build -f dockerfile_nvidia_tecdron_humble -t tecdron_humble .

In order to visualize the gazebo simulation you need to allow non-network local connection to the control list

    xhost local:

Then, just bring in the simulation:

    docker-compose -f docker-compose.yml up

Depends on the branch you clone it will bring different simulations. 

Now you can interact with the simulation in another terminal:

    docker exec -it docker exec -it tecdron_humble_container /bin/bash


## Branchs

### Open_loop branch

In the open_loop branch we have the system working in a open loop,
using foward_controllers in the ros2_control frameworks.
 In this case, the command of wheel velocities are givem 
from the wheel_control_method.cpp file, located in the
robot_description package and the ros2_control
just transfer the values to the simulation.


### closed_loop_outside_ros2_control branch

While in the branch closed_loop_outside_ros2_control we have a half term. The closed loop is working properly but in a separate package named wheel_control and with the ss_control.cpp file. It will feed the forward controller present in the wheels with the proper wheel velocity. So in this step we are calculating the best wheel velocities from outside of the ros2_control framework.

### Main Branch

Finally, in the main branch we have the system running the controller inside the ROS2 control framework.

#### Results

Using the plotjunger we can visualize how well the controller is handling the commands.  

## Control System

The system requires a MIMO controller, Multiple Input Multiple Output.
So the strategy choosen was the robust control for H infinity norm. 
It can handle variations and a certain degree of incertanty in the
parameters and the result is a space state controller. 
The mathematical model and the controller synthesis can be found
in "Matlab Simulations/Controller". And the model can be tested
in a simplified version inside Simulink running the
 "Matlab Simulations/Model/RobustController_Test.slx".

Just a note, the controller had to be discretazed in order to work
properly in ros2, that's why you will see the tustin function 
applied into the controller. 

![closed_loop](images/closed_loop.png)








