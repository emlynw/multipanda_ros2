// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <franka_msgs/msg/franka_model.hpp>
#include <geometry_msgs/msg/twist.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_serl {
  using Eigen::Matrix3d;
  using Matrix4d = Eigen::Matrix<double, 4, 4>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Eigen::Quaterniond;

/**
 * The cartesian impedance example controller implements the Hogan formulation.
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void equilibriumPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  // Ros Communication
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<franka_msgs::msg::FrankaModel>::SharedPtr jacobian_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_speed_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  //  Compliance Parameters
  Matrix6d cartesian_stiffness_;
  Matrix6d cartesian_stiffness_target_;
  Matrix6d cartesian_damping_;
  Matrix6d cartesian_damping_target_;
  Matrix6d Ki_;
  Matrix6d Ki_target_;
  double nullspace_stiffness_;
  double nullspace_stiffness_target_;
  double nullspace_damping_;
  double nullspace_damping_target_;
  double joint1_nullspace_stiffness_;
  double joint1_nullspace_stiffness_target_;
  double filter_params_;
  double delta_tau_max_;
  double delta_tau_max_target_;

  // Clipping Parameters
  Vector3d translational_clip_min_;
  Vector3d translational_clip_max_;
  Vector3d rotational_clip_min_;
  Vector3d rotational_clip_max_;

  // Robot State
  std::string arm_id_;
  Vector7d q_d_nullspace_;
  Vector3d position_d_;
  Vector6d error_;
  Vector6d error_i;
  Quaterniond orientation_d_;
  Matrix4d desired;
  Vector7d joint_lower_limits_; 
  Vector7d joint_upper_limits_;

  // Control Parameters
  const int num_joints = 7;
  int update_counter_ = 0;  // Tracks the current update cycle
  int control_rate_divider_ = 1;  // Number of cycles to skip
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  rclcpp::Time start_time_;

  // Replace the declare_parameters() function with:
  void declare_parameters() {
      auto node = get_node();
      node->declare_parameter("translational_stiffness", 2000.0);
      node->declare_parameter("rotational_stiffness", 200.0);
      node->declare_parameter("translational_damping", 89.0);
      node->declare_parameter("rotational_damping", 10.0);
      node->declare_parameter("joint1_nullspace_stiffness", 80.0);
      node->declare_parameter("nullspace_stiffness", 0.5);
      node->declare_parameter("nullspace_damping", 2.0);
      node->declare_parameter("filter_params", 0.001);
      node->declare_parameter("translational_Ki", 0.0);
      node->declare_parameter("rotational_Ki", 0.0);
      node->declare_parameter("translational_clip_x", 0.01);
      node->declare_parameter("translational_clip_neg_x", 0.01);
      node->declare_parameter("translational_clip_y", 0.01);
      node->declare_parameter("translational_clip_neg_y", 0.01);
      node->declare_parameter("translational_clip_z", 0.01);
      node->declare_parameter("translational_clip_neg_z", 0.01);
      node->declare_parameter("rotational_clip_x", 0.03);
      node->declare_parameter("rotational_clip_neg_x", 0.03);
      node->declare_parameter("rotational_clip_y", 0.03);
      node->declare_parameter("rotational_clip_neg_y", 0.03);
      node->declare_parameter("rotational_clip_z", 0.03);
      node->declare_parameter("rotational_clip_neg_z", 0.03);
      node->declare_parameter("delta_tau_max_", 0.5);
  }
};

}  // namespace franka_serl