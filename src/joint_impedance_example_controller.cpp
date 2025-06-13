// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool JointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
if (state_interface == nullptr) {
  ROS_ERROR_STREAM("JointImpedance: Error getting state interface from hardware");
  return false;
}
try {
  state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
      state_interface->getHandle(arm_id + "_robot"));
} catch (const hardware_interface::HardwareInterfaceException& e) {
  ROS_ERROR_STREAM("JointImpedance: Exception getting state handle: " << e.what());
  return false;
}


  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  q_cmd_sub_ = node_handle.subscribe<sensor_msgs::JointState>(
               "command", 1,
               &JointImpedanceExampleController::commandCallback, this);

  std::fill(dq_measured_last_.begin(), dq_measured_last_.end(), 0.0);
  return true;
}

void JointImpedanceExampleController::commandCallback(const sensor_msgs::JointStateConstPtr& msg) {
  if (msg->position.size() != 7) return;
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  for (size_t i = 0; i < 7; ++i) {
    q_des_[i]  = msg->position[i];
    dq_des_[i] = (msg->velocity.size()==7) ? msg->velocity[i] : 0.0;
  }
}


void JointImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // starting()
  auto robot_q = state_handle_->getRobotState().q;
  // measured q
  for (size_t i = 0; i < 7; ++i) 
  {
    q_des_[i]  = robot_q[i];
    dq_des_[i] = 0.0;
  }
}

void JointImpedanceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {


  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.90;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }
  // double dt = period.toSec();
  // double cutoff = 30.0;  // could be more smaller 
  // for (size_t i = 0; i < 7; ++i) 
  // {
  //   dq_filtered_[i] = franka::lowpassFilter(dt, robot_state.dq[i], dq_measured_last_[i], cutoff);
  //   dq_measured_last_[i] = dq_filtered_[i];  

  // }

    std::array<double, 7> qd_local, dqd_local;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    qd_local  = q_des_;
    dqd_local = dq_des_;
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) 
  {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] + k_gains_[i] * (qd_local[i]  - robot_state.q[i]) + d_gains_[i] * (dqd_local[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

}

std::array<double, 7> JointImpedanceExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerBase)
