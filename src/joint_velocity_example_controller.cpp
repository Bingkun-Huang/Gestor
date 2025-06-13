// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
      state_interface->getHandle(arm_id + "_robot"));

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 3) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  command_sub_ = node_handle.subscribe<sensor_msgs::JointState>("command", 1, &JointVelocityExampleController::commandCallback, this);

  commanded_velocities_.fill(0.0);
  ROS_INFO("JointVelocityExampleController: Ready to receive joint velocity commands on 'command' topic.");
  
  last_command_time_ = ros::Time::now();

  return true;
}

// void JointVelocityExampleController::commandCallback(const sensor_msgs::JointStateConstPtr& msg) {
//   if (msg->velocity.size() != 7) 
//   {
//     ROS_WARN_THROTTLE(1.0, "Received JointState with invalid velocity size!");
//     return;
//   }
//   std::lock_guard<std::mutex> lock(cmd_mutex_);
//   for (size_t i = 0; i < 7; ++i) {
//     commanded_velocities_[i] = msg->velocity[i];
//   }
//   ROS_INFO("Received: qd1: %f, qd2: %f, qd3: %f, qd4: %f, qd5: %f, qd6: %f, qd7: %f",
//              msg->velocity[0], msg->velocity[1], msg->velocity[2],
//              msg->velocity[3], msg->velocity[4], msg->velocity[5], msg->velocity[6]);
// }

void JointVelocityExampleController::commandCallback(const sensor_msgs::JointStateConstPtr& msg) 
{
  if (msg->position.size() != 7) return;
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  for (size_t i = 0; i < 7; ++i) 
  {
    q_des_[i] = msg->position[i];
  }
  last_command_time_ = ros::Time::now();
  ROS_INFO("Received: q1: %f, q2: %f, q3: %f, q4: %f, q5: %f, q6: %f, q7: %f",
             msg->position[0], msg->position[1], msg->position[2],
             msg->position[3], msg->position[4], msg->position[5], msg->position[6]);
  has_command_ = true;
}


void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

// void JointVelocityExampleController::update(const ros::Time& /* time */,
//                                             const ros::Duration& period) {

//   // elapsed_time_ += period;
//   // if message is empty, keep the velocities at zero
//   if (commanded_velocities_.empty()) {
//     for (auto& joint_handle : velocity_joint_handles_) {
//       joint_handle.setCommand(0.0);
//     }
//     return;    
//   }                              
//   std::array<double, 7> v_cmd;
//   {
//     std::lock_guard<std::mutex> lock(cmd_mutex_);
//     v_cmd = commanded_velocities_;
//   }

//   for (size_t i = 0; i < 7; ++i) 
//   {
//     velocity_joint_handles_[i].setCommand(v_cmd[i]);
//   }

//   // ros::Duration time_max(8.0);
//   // double omega_max = 0.1;
//   // double cycle = std::floor(
//   //     std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
//   //                        time_max.toSec()));
//   // double omega = cycle * omega_max / 2.0 *
//   //                (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

//   // for (auto joint_handle : velocity_joint_handles_) {
//   //   joint_handle.setCommand(omega);
//   // }
// }

void JointVelocityExampleController::update(const ros::Time&, const ros::Duration& period) {

    if ((ros::Time::now() - last_command_time_).toSec() > command_timeout_)
    {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0.0);
    }
    has_command_ = false;
    return;
  }
  
  if (!has_command_) {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0.0);
    }
    return;
  }

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> q_current = robot_state.q;

  std::array<double, 7> dq_cmd;
  static std::array<double, 7> dq_cmd_filter = {0,0,0,0,0,0,0};
  double alpha = 0.99;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      dq_cmd[i] = (q_des_[i] - q_current[i]) * k_d;
      dq_cmd_filter[i] = (1 - alpha) * dq_cmd_filter[i] + alpha * dq_cmd[i];
    }
  }

  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(dq_cmd_filter[i]);
  }
}


void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
