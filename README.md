# cpwalker-ros_simulation
This repository is based on the [cpwalker-ros](https://github.com/ricardocmello/cpwalker-ros) repository and aims to create a **simulated ROS environment for the CPWalker platform** where you can test ROS nodes without connecting to the real robot. In this way, it is possible to develop new strategies directly in the ROS environment, which entails minimal changes when they are to be deployed on the real platform.

The virtual environment is based on the **`exo_model_node`**, a ROS node that contains the transfer functions of the joints and simulates the dinamics of the exoskeleton of the robotic platform. Inside the _exo_model_ folder you will find a Simulink file with which the **`exo_model_node`** can be generated. This Simulink file has been developed using the Matlab and to transform it into a standalone ROS Node you must follow the instructions of the [Generate a Standalone ROS Node from Simulink](https://es.mathworks.com/help/ros/ug/generate-a-standalone-ros-node-from-simulink.html).

* **RESUME**

The typical connections of the exoskeleton system look like the left _Figure 1_, in which the processors (Raspberries) are interconnected via a ROS network through which they exchange messages via topics (see right _Figure 1_) about the sensor's data that they obtain through wired connections to physical sensors. This data is processed by the Raspberries using the control ROS node which generates the desired voltage that must be sent to the motors to move the exoskeleton to the desired position.  

The exoskeleton model node substitutes the function of the physical system and the acquisition and processing node of the right _Figure 1_. This node subscribes to the Vout topics, calculates the behavior of the system and publish through ROS topics the estimated angle position of each joint, see left _Figure 2_. The **`simulated_exo_control_node`** subscribes to those topics, calculates the needed Vout to follow the desired path and publishes that value through Vout ROS topics, see right _Figure 2_. 

<p align="center">
  <image src = "images/Physical_conections_cpwalker_exo.png" width = "500" />
   <image src = "images/rosgraph_acquisitio_processing_control.png" width = "400" />
</p>
<p align="center">
  <b>Figure 1. Left physical connection to the robot, right ROS topics communication between the nodes.</b>
</p>

<p align="center">
  <image src = "images/exo_model_node.png" width = "400" />
  <image src = "images/simulation_exo_control_node.png" width = "400" />
</p>
<p align="center">
  <b>Figure 2. Left, ROS exo_model_node. Right simulated_control_node </b>
</p>

