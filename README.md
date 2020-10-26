# cpwalker-ros_simulation
This repository aims to create a **simulated ROS environment for the CPWalker platform** where you can test ROS nodes platform without connecting to the real robot. In this way it is possible to develop new strategies directly in the ROS environment with minimal changes compared to those that will be deployed in the real platform.

The virtual environment is based on the **`exo_model_node`** a ROS node that simulates the dinamics of the exoskeleton of the robotic platform. Inside the exo_model folder you will find a Simulink file through which the exo_model_node can be generated. Whithin this Simulink are the transfer functions of the joints of the CPWalker exoskelton that represent the mathematical models of the joints that defines the exoskeleton dinamics. This Simulink file has been developed using the Matlab ROS toolbox and in order to transform it into a standalone ROS Node you must follow the instructions of the [Generate a Standalone ROS Node from Simulink](https://es.mathworks.com/help/ros/ug/generate-a-standalone-ros-node-from-simulink.html).

* **RESUME**
----

The typical connections of the exoskeleton system look like the Figure 1, in which the processors (Raspberries) are interconnected via a ROS network through which they exchange messages via topics (see Figure 2) about the sensors data that they obtain through wired connections to physical sensors. This data is processed by the Raspberries using the control ROS node which generates the desired voltage that must be sent to the motors to move the exoskeleton to the desired position.  

The exoskeleton model node substitutes the function of the physical system and the acquisition and processing node of the Figure 2. This node subscribes to the Vout topics generated by the processors, calculates the behavior of the system under said voltage and publish the estimated angle position of each joint, see Figure 3. 

![alt text](https://github.com/Romerozabal/cpwalker-ros_simulation/tree/main/Physical_conections_cpwalker_exo_1.png "Figure 1. Physical connection to the robot.")
![alt text](https://github.com/Romerozabal/cpwalker-ros_simulation/tree/main/rosgraph_acquisitio_processing_control.png "Figure 2. ROS topics communication between the nodes.")
![alt text](https://github.com/Romerozabal/cpwalker-ros_simulation/tree/main/exo_model_node_1.png "Figure 3. ROS exo model node, subscribes to Vout and publish the angle of each joint.")
