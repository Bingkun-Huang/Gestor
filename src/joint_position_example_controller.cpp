// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_example_controllers/tsia_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
    return false;
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  trajectory_subscriber_ = node_handle.subscribe("command", 1, &JointPositionExampleController::trajectoryCallback, this);
  ROS_INFO("JointPositionExampleController: Ready to receive joint trajectory on 'command' topic.");
  return true;
}

void JointPositionExampleController::starting(const ros::Time& time) {
  has_new_trajectory_ = false;
  trajectory_start_time_ = time;

  // 初始化当前关节位置
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }

  // 保持静止
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(initial_pose_[i]);
  }

  ROS_INFO("JointPositionExampleController: Controller started with initial position.");
}

void JointPositionExampleController::update(const ros::Time& time, const ros::Duration& /* period */) {
  if (!has_new_trajectory_) {
      for (size_t i = 0; i < 7; ++i) 
      {
      position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    return;
  }

  ros::Duration elapsed = time - trajectory_start_time_;
  double t = elapsed.toSec();
  ROS_INFO_THROTTLE(1.0, "Trajectory time since start: %.2f", t);

  // 找到当前时间对应的轨迹点
  size_t i = 0;
  while (i + 1 < current_trajectory_.points.size() &&
         current_trajectory_.points[i + 1].time_from_start.toSec() < t) {
    ++i;
  }

  const auto& pt = current_trajectory_.points[i];
  if (pt.positions.size() != 7) {
    ROS_WARN_THROTTLE(1.0, "JointPositionExampleController: Received point with wrong joint count");
    return;
  }

  for (size_t j = 0; j < 7; ++j) {
    position_joint_handles_[j].setCommand(pt.positions[j]);
  }
}

void JointPositionExampleController::trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
  if (msg->points.empty()) {
    ROS_WARN("JointPositionExampleController: Received empty trajectory.");
    return;
  }
  current_trajectory_ = *msg;
  trajectory_start_time_ = ros::Time::now();
  has_new_trajectory_ = true;

  ROS_INFO("JointPositionExampleController: Received trajectory with %lu points.", msg->points.size());
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)

