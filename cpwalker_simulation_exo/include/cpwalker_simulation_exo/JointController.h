/*
 * \file control_node.cpp
 *
 * \date Created on march 10, 2020
 * \author Pablo Romero Sorozábal <p.romero.sorozabal@gmail.com>
 */

#ifndef CPWALKER_SIMULATION_EXO_JOINTCONTROLLER_H_
#define CPWALKER_SIMULATION_EXO_JOINTCONTROLLER_H_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "cpwalker_simulation_exo/PID.h"

class JointController {
public:

  JointController();
  JointController(std::string joint, std::vector<bool> hardware, int sampling_frequency);

  // ROS Callback functions
  void angleCallback(const std_msgs::Float64& msg);
  void torqueCallback(const std_msgs::Float64& msg);
  void rawGaugeCallback(const std_msgs::UInt16& msg);
  void fsr1Callback(const std_msgs::Float64& msg);
  void fsr2Callback(const std_msgs::Float64& msg);

  // Return a pointer to the subscriber of the joint.
  inline ros::Subscriber& getAngleSubscriber() {return angle_subscriber_;}
  inline ros::Subscriber& getTorqueSubscriber() {return torque_subscriber_;}
  inline ros::Subscriber& getRawGaugeSubscriber() {return raw_gauge_subscriber_;}
  inline ros::Subscriber& getFsr1Subscriber() {return fsr1_subscriber_;}
  inline ros::Subscriber& getFsr2Subscriber() {return fsr2_subscriber_;}

  // Return a pointer to the publisher of the joints controlled Vout:
  inline ros::Publisher& getVoutPublisher() {return publisher_controlled_Vout_;}

  // Set private variables values.
  inline void setControlledVoltage(float controlled_voltage) {controlled_voltage_ =  controlled_voltage_pub_.data = controlled_voltage; }

  // Publish Voltage.
  inline void pubVout() {publisher_controlled_Vout_.publish(controlled_voltage_pub_);}

  // Get private variables values.
  inline float getAngle() const {return angle_;}
  inline float getTorque() const {return torque_;}
  inline uint16_t getRawGauge() const {return raw_gauge_;}
  inline float getFsr1() const {return fsr1_;}
  inline float getFsr2() const {return fsr2_;}
  inline std::string getJoinName() { return joint_; }
  inline float getControlledVoltage() { return controlled_voltage_; }
  inline float getSetPosition() {return set_point_;}

  // Updates next trajectory position and error.
  void updateSetPoint(float set_point);

  // Returns True if the joint is in its start position, also updates the control voltage signal to move joint to start position.
  bool setUpJointsControl();

  // Updates the control voltage signal for the trajectory/postion control.
  void positionControl();

  // Updates the control voltage signal to accomplish the impedance control
  void impedanceControl(char impedance_level);

  // Returns the level of impedance
  float impedanceGenerator(float stiffness, float damping, float inertia);

  // Prevents mechanical damage, controlls voltage if the safety limits have been reached.
  void mechanicalLimits();

  ~JointController();

private:
  std::string joint_;     //!< Name of the joint to be control.
  std::vector<bool> hardware_;  //!< List of the hardware componets available in the joint.
  int sampling_frequency_;  //!< Sampling frequency of the system.

  ros::Subscriber angle_subscriber_;
  ros::Subscriber torque_subscriber_;
  ros::Subscriber raw_gauge_subscriber_;
  ros::Subscriber fsr1_subscriber_;
  ros::Subscriber fsr2_subscriber_;

  ros::Publisher publisher_controlled_Vout_;


  float controlled_voltage_;
  std_msgs::Float64 controlled_voltage_pub_;

  float set_point_; //!< Set point of position in the joint trajectory.
  float position_error_; //!< Difference between set_position and current position of the joint

  float impedance_;  //!< Desired impedance
  float torque_error_; //!< Difference between impedance and current torque value

  float set_velocity_;  //!< Set point of velocity desired.

  float angle_;         //!< Postion of the potentiometer.
  float last_angle_;    //!< Previous postion of the potentiometer.
  float velocity_;      //!< Angular velocity of the joint.
  float last_velocity_; //!< Previous angular velocity.
  float acceleration_;  //!< Acceleration of the joint. (as it is a rotation movement the acceleration is a "normal acceleration")
  float torque_;         //!< Value of the gauge.
  uint16_t raw_gauge_;
  float fsr1_;          //!< Value of the fsr1.
  float fsr2_;          //!< Value of the fsr2.

  PID position_pid_; //!< Position PID controller for trajectory control

  PID low_impedance_pid_; //!< Toque PID controller for low impedance control
  PID medium_impedance_pid_; //!< Toque PID controller for medium impedance control
  PID high_impedance_pid_; //!< Toque PID controller for high impedance control

  float low_stiffness_; //!< Impedance position constant
  float low_damping_;   //!< Impedance velocity constant
  float low_inertia_;   //!< Impedance acceleration constant

  float medium_stiffness_; //!< Impedance position constant
  float medium_damping_;   //!< Impedance velocity constant
  float medium_inertia_;   //!< Impedance acceleration constant

  float high_stiffness_; //!< Impedance position constant
  float high_damping_;   //!< Impedance velocity constant
  float high_inertia_;   //!< Impedance acceleration constant

  float last_position;
};

#endif  // CPWALKER_SIMULATION_EXO_JOINTCONTROLLER_H_
