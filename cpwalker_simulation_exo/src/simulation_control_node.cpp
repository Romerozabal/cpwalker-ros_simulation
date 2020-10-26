/*
 * \file control_node.cpp
 *
 * \date Created on feb 26, 2021
 * \author Pablo Romero Sorozábal <p.romero.sorozabal@gmail.com>
 */

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <array>
#include <vector>
#include "cpwalker_simulation_exo/JointController.h"
#include <math.h>
#include <fstream>

//Global lists:
std::string joints_global[4]  = {"right_knee", "left_knee", "right_hip", "left_hip"};
std::string hw_names[4]       = {"Potentiometer", "Gauge", "FSR1", "FSR2"};
std::string processed_data[4] = {"_angle", "_gauge", "_fsr1", "_fsr2"};

//Panic variable.
bool stop_exo = 0;

//Panic listener function:
void stopMotionExoCallBack (const std_msgs::Bool::ConstPtr& stop) {
  if (stop->data == true) {
    stop_exo = stop->data;
    ROS_WARN("Stop exo joints.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "exo_control_node");
  ros::NodeHandle nh;

  int size_trajectory_vector = 0;
  int sampling_frequency;                             //sampling_frequency of the control
  int step_time = 2;  //Predefined step time
  int steps_number = 2; //Predefined number of steps
  std::vector<float> hip_trajectory_vector;           // Hip trajectory vector
  std::vector<float> knee_trajectory_vector;          // Knee trajctory vector
  std::vector<bool> joints_available;                 // List of availabe joints
  std::vector<std::vector<bool>> hardware_available;  // List of availabe hardware per joint
  hardware_available.resize(4);                       // Hardware list for each joint

  // Get hip-knee trajectory vector size from trajectory.yaml file
  if (nh.hasParam("trajectory/size"))
    nh.getParam("trajectory/size", size_trajectory_vector);
  else
    ROS_ERROR("PARAMETER missing: size of vector in trajectory/");

  //Get control frequency from hardware.yaml file
  if (nh.hasParam("exo_hw/sampling_frequency"))
    nh.getParam("exo_hw/sampling_frequency",sampling_frequency);
  else
    ROS_ERROR("PARAMETER missing: sampling_frequency in exo_hw/");

  // Check joints and hardware available in the hardware.yaml file
  for (int i = 0; i<4; i++) {
    if(nh.hasParam("exo_hw/joints/" + joints_global[i])) {//Joint.
      joints_available.push_back(true);
      ROS_INFO("%s selected:", joints_global[i].c_str());
      for (int j = 0; j<4;j++) {
        bool has_component;
        nh.getParam("exo_hw/joints/" + joints_global[i] + "/" + hw_names[j], has_component);
        if (has_component) {
          hardware_available[i].push_back(true);
          ROS_INFO(" - %s selected.", hw_names[j].c_str());
        } else {
          hardware_available[i].push_back(false);
          ROS_WARN(" - %s NOT selected.", hw_names[j].c_str());
        }
      }
    } else {
      joints_available.push_back(false);
      ROS_WARN("%s NOT selected.", joints_global[i].c_str());
      for (int j = 0; j<4;j++) {
        hardware_available[i].push_back(false);
      }
    }
  }

  // Get hip and knee trajectories in the trajectory.yaml file
  for (int i = 0; i <= size_trajectory_vector; i++) {
    if (nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i))) {
      if (nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i) + "/hip") && nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i) +"/knee")) {
        float position;
        nh.getParam("trajectory/hip_knee_positions/" + std::to_string(i) + "/hip", position);
        hip_trajectory_vector.push_back(position);
        nh.getParam("trajectory/hip_knee_positions/" + std::to_string(i) +"/knee", position);
        knee_trajectory_vector.push_back(position);
      } else {
        ROS_ERROR("PARAMETER missing: Joint positions in hip_knee_positions/");
        break;
      }
    } else {
      ROS_ERROR("PARAMETER missing: Joint positions in hip_knee_positions/");
      break;
    }
  }

  // Publish controlled voltage applied to the right knee

  // Generate the Publisher and Subscribe of each available joint to the angle and gauge topics and publish the controlled voltage:
  std::vector<JointController *> joint_controller(4);
  for (int i = 0; i<4;i++) {
    joint_controller[i] = new JointController(joints_global[i], hardware_available[i], sampling_frequency);
    if (joints_available[i]) {
      for (int j = 0; j<4;j++) {
        if (hardware_available[i][j]) {
          if (j==0) {
            joint_controller[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::angleCallback, joint_controller[i]);
            joint_controller[i]->getVoutPublisher() = nh.advertise<std_msgs::Float64>("/" + joints_global[i] + "_Vout",1000);
          } else if (j==1) {
            joint_controller[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::angleCallback, joint_controller[i]);
            joint_controller[i]->getVoutPublisher() = nh.advertise<std_msgs::Float64>("/" + joints_global[i] + "_Vout",1000);
          } else if (j==2) {
            joint_controller[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::angleCallback, joint_controller[i]);
            joint_controller[i]->getVoutPublisher() = nh.advertise<std_msgs::Float64>("/" + joints_global[i] + "_Vout",1000);
          } else if (j==3) {
            joint_controller[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::angleCallback, joint_controller[i]);
            joint_controller[i]->getVoutPublisher() = nh.advertise<std_msgs::Float64>("/" + joints_global[i] + "_Vout",1000);
          }
        }
      }
    }
  }

  // Subscribe to the panic topic
  ros::Subscriber stop_motion_exo = nh.subscribe("/stop_motion_exo", 100, &stopMotionExoCallBack);

  // SPI communication
  /*
  SPI spi;
  if (!spi.init()){
    ROS_ERROR("SPI ERROR");
    return 1;
  }
  */

  // Stop all joints.
  float Vout_zero = 0;
  /*
  for (int i = 0; i < 4; i++) {
    spi.sendData(joints_global[i], Vout_zero);
  }
  */

  // Type of therapy
  char therapy_type = 's';
  char impedance_level;
  char start_gait;
  // Start conditions
  bool ready[4] = { false, false, false, false };
  char prepare;
  bool continue_therapy = true;

  // Control loop variables
  ros::Rate loop_rate(sampling_frequency);
  int sampling_time = 0;
  int iterator_right_joints = 0;
  int iterator_left_joints = static_cast<int> (size_trajectory_vector / 2); // Defase the position of the left joints

  // Save therapy info.
  const char *path="/home/pi/Documents/Test.data";
  std::ofstream testFile(path, std::ios::trunc);
  testFile.open(path,std::ofstream::out | std::ofstream::trunc);

  const char *path_torque="/home/pi/Documents/Torque.data";
  std::ofstream torqueFile(path_torque, std::ios::trunc);
  torqueFile.open(path,std::ofstream::out | std::ofstream::trunc);

  const char *path_time="/home/pi/Documents/Time.data";
  std::ofstream timeFile(path_time, std::ios::trunc);
  timeFile.open(path,std::ofstream::out | std::ofstream::trunc);

  const char *path_pot_position="/home/pi/Documents/PotPosition.data";
  std::ofstream potPositionFile(path_pot_position, std::ios::trunc);
  potPositionFile.open(path,std::ofstream::out | std::ofstream::trunc);

  const char *path_ref_position="/home/pi/Documents/RefPosition.data";
  std::ofstream refPositionFile(path_ref_position, std::ios::trunc);
  refPositionFile.open(path,std::ofstream::out | std::ofstream::trunc);

  const char *path_voltage="/home/pi/Documents/Voltage.data";
  std::ofstream voltageFile(path_voltage, std::ios::trunc);
  voltageFile.open(path,std::ofstream::out | std::ofstream::trunc);

  for (int i = 0; i < 4; i++) {
    if(joints_available[i] && !hardware_available[i][0])
      ROS_ERROR(" The position sensor of the %s has not been detected, it is not possible to perform any therapy without this sensor.",joints_global[i].c_str());
  }

  // CONTROL LOOP
  while (ros::ok() && continue_therapy && !stop_exo ) {
    // Time conditions:
    if (sampling_time >= step_time * sampling_frequency * steps_number) {
      sampling_time = 0;
      therapy_type = 's';

      //Close file with the info of the therapy.
      testFile.close();
      torqueFile.close();
      timeFile.close();
      potPositionFile.close();
      refPositionFile.close();
      voltageFile.close();

      //Put initial condidions and stop the joint motors.
      for (int i = 0; i < 4; i++) {
        //spi.sendData(joints_global[i], Vout_zero);
        ready[i] = false;
        joint_controller[i]->setControlledVoltage(Vout_zero);
        joint_controller[i]->pubVout();
      }
    }

    // Change set points depending on the duration of the step:
    if (sampling_time % (step_time * sampling_frequency / size_trajectory_vector) == 0) {
      iterator_right_joints = iterator_right_joints + 1;
      iterator_left_joints = iterator_left_joints + 1;
    }

    // Loop conditions for joint position iterators
    if (iterator_right_joints == static_cast<int>(size_trajectory_vector))
      iterator_right_joints = 0;
    if (iterator_left_joints == static_cast<int>(size_trajectory_vector) )
      iterator_left_joints = 0;

    // Update set points of the Joints:
    for (int i = 0; i<4;i++) {
      if (joints_available[i]) {
        switch (i) {
          case 0: // RIGHT KNEE
            joint_controller[i]->updateSetPoint(knee_trajectory_vector[iterator_right_joints]);
            break;
          case 1: // LEFT KNEE
            joint_controller[i]->updateSetPoint(knee_trajectory_vector[iterator_left_joints]);
            break;
          case 2: // RIGHT HIP
            joint_controller[i]->updateSetPoint(hip_trajectory_vector[iterator_right_joints]);
            break;
          case 3: // LEFT HIP
            joint_controller[i]->updateSetPoint(hip_trajectory_vector[iterator_left_joints]);
            break;
        };
      }
    }

    // Control movement depending on therapy type
    switch (therapy_type) {
      case 's':
        // Prerare joints, move joints to their start positions
        sampling_time = 0;
        iterator_right_joints = 0;
        iterator_left_joints = static_cast<int> (size_trajectory_vector / 2);

        // Starting point
        if (!ready[0] && !ready[1] && !ready[2] && !ready[3]) {
          do {
            std::cout << "**** Do you want to start the therapy? ****" << '\n' << "*Important information: " << '\n';
            for (int i = 0; i<4;i++) {
              if (!joints_available[i]) {
                std::cout << " " << joints_global[i] << " is NOT detected." << '\n';
              } else {
                std::cout << " " << joints_global[i] << " is detected." << '\n';
                for (int j = 0; j<4;j++) {
                  if (!hardware_available[i][j])
                    std::cout << "  -" << hw_names[j] << " is not detected." << '\n';
                }
              }
            }
            std::cout  << '\n' << "Answer (y: yes | n: no/exit ): ";
            std::cin >> prepare;
            if (prepare == 'y'){
              continue_therapy = true;
              therapy_type = 's'; // 's' : Set up joints position, move them to initial positios
              std::cout << '\n' << "Moving joints to their start positions..." << '\n' << '\n' << '\n';
            } else {
              continue_therapy = false;
            }
          } while(prepare != 'y' && prepare != 'n');
        }

        // If joints are in their start positions ask for the type of therapy.
        if (ready[0] && ready[1] && ready[2] && ready[3]) {
          do {
            std::cout  << '\n' << "**** What type of therapy do you desire? ****" << '\n' << '\n' << "Answer ( p: position control | i: impedance control ): ";
            std::cin >> therapy_type;
          } while ( therapy_type != 'p' && therapy_type != 'i');

          if (therapy_type == 'i') {
            do {
              std::cout  << '\n' << "**** What impedance intensity do you want? ****" << '\n' << '\n' << "Answer ( 1: low | 2: medium | 3: high ): ";
              std::cin >> impedance_level;
            } while( impedance_level != '1' && impedance_level != '2' && impedance_level != '3' && impedance_level != 'e');
          }

          do {
            std::cout << '\n' << "**** Velocity per step in seconds? ****" << '\n' << '\n' << "Answer: ";
            std::cin.clear(); // clear the error flags
            std::cin.ignore(INT_MAX, '\n'); // discard the row
          } while (!(std::cin >> step_time) || step_time < 2);

          do {
            std::cout << '\n' << "**** How many steps do you want to perform? ****" << '\n' << '\n' << "Answer: ";
            std::cin.clear(); // clear the error flags
            std::cin.ignore(INT_MAX, '\n'); // discard the row
          } while (!(std::cin >> steps_number));

          do {
            std::cout << '\n' << "**** Start gait? (y,n) ****" << '\n' << '\n' << "Answer (y: yes | n: no ): ";
            std::cin >> start_gait;
            // Reset to initial conditions
            if (start_gait == 'n') {
              therapy_type = 's';
              for (int i = 0; i < 4; i++) {
                ready[i] = false;
                joint_controller[i]->setControlledVoltage(Vout_zero);
              }
            } else {
              testFile.open(path,std::ofstream::out | std::ofstream::trunc);
              torqueFile.open(path_torque,std::ofstream::out | std::ofstream::trunc);
              timeFile.open(path_time,std::ofstream::out | std::ofstream::trunc);
              potPositionFile.open(path_pot_position,std::ofstream::out | std::ofstream::trunc);
              refPositionFile.open(path_ref_position,std::ofstream::out | std::ofstream::trunc);
              voltageFile.open(path_voltage,std::ofstream::out | std::ofstream::trunc);
              if(!testFile.is_open() || !torqueFile.is_open() || !timeFile.is_open() || timeFile.is_open() || potPositionFile.is_open() || refPositionFile.is_open() || voltageFile.is_open())
                  std::cout << "ERROR no info can be saved in the file." << '\n';
              std::cout << '\n' << "Moving joints ..." << '\n' << '\n' << '\n';
            }
          } while(start_gait != 'y' && start_gait != 'n');
        } else {
          // Check what jointsare not in their start positions
          for (int i = 0; i<4;i++) {
            if (joints_available[i]) {
              if (hardware_available[i][0]) {
                if (!ready[i])
                  ready[i] = joint_controller[i]->setUpJointsControl();
              }
            } else
              ready[i] = true;
          }
        }
        break;
      case 'p':
        // Position control of the joints
        for (int i = 0; i<4;i++) {
          // Trajectory control
          if (joints_available[i])
            joint_controller[i]->positionControl();
        }
        break;
      case 'i':
        // Impedance control of the joints
        for (int i = 0; i<4;i++) {
          if (joints_available[i]) {
            if (!hardware_available[i][1]) {
              ROS_ERROR("The torque sensor has not been detected, the impedance control cannot be accomplished without the position sensor.");
            } else {
              // Impedance control
              joint_controller[i]->impedanceControl(impedance_level);
            }
          }
        }
        break;
    }

    // File information
    std::string file_info = std::to_string(sampling_time) + " " + std::to_string(joint_controller[0]->getSetPosition()) + " " + std::to_string(joint_controller[0]->getAngle()) + " " + std::to_string(joint_controller[0]->getTorque()) + " " +  std::to_string(joint_controller[0]->getRawGauge()) + '\n';
    testFile << file_info;
    potPositionFile << std::to_string(joint_controller[0]->getAngle()) + " ";
    refPositionFile << std::to_string(joint_controller[0]->getSetPosition()) + " ";
    torqueFile << std::to_string(joint_controller[0]->getTorque()) + " ";
    timeFile << std::to_string(sampling_time) + " ";
    voltageFile << std::to_string(joint_controller[0]->getControlledVoltage()) + " ";

    // Move joints
    for (int i = 0; i<4;i++) {
      if (joints_available[i]) {
        // Check mechanical limits
        joint_controller[i]->mechanicalLimits();
        // Send data
        //spi.sendData(joint_controller[i]->getJoinName(), joint_controller[i]->getControlledVoltage());
        joint_controller[i]->pubVout();
      }
    }

    //Count time
    sampling_time = sampling_time + 1;
    ros::spinOnce();
    loop_rate.sleep();
  }


  //Put all the SPI outputs to 0V.
  /*
  for (int i = 0; i < 4; i++) {
    spi.sendData(joints_global[i], Vout_zero);
  }
  spi.end();
  */

  joint_controller.clear();


  // Delete ROS params
  nh.deleteParam("exo_hw");
  nh.deleteParam("trajectory");


  return 0;
}
