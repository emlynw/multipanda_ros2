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

#include <franka_serl/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <franka/model.h>
#include <franka_msgs/msg/franka_model.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <rclcpp/parameter.hpp>

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


namespace franka_serl {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // should be model interface
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    declare_parameters();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  auto node = get_node();

  arm_id_ = node->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));


  // Initialize parameter targets from YAML
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3) = 
      node->get_parameter("translational_stiffness").as_double() * Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3) = 
      node->get_parameter("rotational_stiffness").as_double() * Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner(3, 3) = 
      node->get_parameter("translational_damping").as_double() * Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3) = 
      node->get_parameter("rotational_damping").as_double() * Matrix3d::Identity();

  nullspace_stiffness_target_ = node->get_parameter("nullspace_stiffness").as_double();
  nullspace_damping_target_ = node->get_parameter("nullspace_damping").as_double();
  joint1_nullspace_stiffness_target_ = node->get_parameter("joint1_nullspace_stiffness").as_double();
  delta_tau_max_target_ = node->get_parameter("delta_tau_max_").as_double();

  Ki_target_.setIdentity();
  Ki_target_.topLeftCorner(3, 3) = 
      node->get_parameter("translational_Ki").as_double() * Matrix3d::Identity();
  Ki_target_.bottomRightCorner(3, 3) = 
      node->get_parameter("rotational_Ki").as_double() * Matrix3d::Identity();

  translational_clip_min_ = Vector3d(
      -node->get_parameter("translational_clip_neg_x").as_double(),
      -node->get_parameter("translational_clip_neg_y").as_double(),
      -node->get_parameter("translational_clip_neg_z").as_double());
  translational_clip_max_ = Vector3d(
      node->get_parameter("translational_clip_x").as_double(),
      node->get_parameter("translational_clip_y").as_double(),
      node->get_parameter("translational_clip_z").as_double());
  rotational_clip_min_ = Vector3d(
      -node->get_parameter("rotational_clip_neg_x").as_double(),
      -node->get_parameter("rotational_clip_neg_y").as_double(),
      -node->get_parameter("rotational_clip_neg_z").as_double());
  rotational_clip_max_ = Vector3d(
      node->get_parameter("rotational_clip_x").as_double(),
      node->get_parameter("rotational_clip_y").as_double(),
      node->get_parameter("rotational_clip_z").as_double());

  filter_params_ = node->get_parameter("filter_params").as_double();

  // Set up parameter callback
  params_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&CartesianImpedanceController::parametersCallback, this, std::placeholders::_1));

  pose_subscriber_ = this->get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/franka/goal_pose", 1, std::bind(&CartesianImpedanceController::equilibriumPoseCallback, this, std::placeholders::_1));

  // Add publishers for Jacobian and Cartesian speed
  jacobian_publisher_ = this->get_node()->create_publisher<franka_msgs::msg::FrankaModel>("/franka/jacobian", 1);
  cartesian_speed_publisher_ = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("/franka/cartesian_speed", 1);
  joint_lower_limits_ << -2.75, -1.58, -2.75, -2.99, -2.75, -0.11, -2.75;
  joint_upper_limits_ << 2.75, 1.58, 2.75, -0.15, 2.75, 3.66, 2.75;
        
  return CallbackReturn::SUCCESS;
}

// Add parameter callback implementation
rcl_interfaces::msg::SetParametersResult CartesianImpedanceController::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto& parameter : parameters) {
    const std::string& name = parameter.get_name();
    
    if (name == "translational_stiffness") {
      cartesian_stiffness_target_.topLeftCorner(3, 3) = 
          parameter.as_double() * Matrix3d::Identity();
    } else if (name == "rotational_stiffness") {
      cartesian_stiffness_target_.bottomRightCorner(3, 3) = 
          parameter.as_double() * Matrix3d::Identity();
    } else if (name == "translational_damping") {
      cartesian_damping_target_.topLeftCorner(3, 3) = 
          parameter.as_double() * Matrix3d::Identity();
    } else if (name == "rotational_damping") {
      cartesian_damping_target_.bottomRightCorner(3, 3) = 
          parameter.as_double() * Matrix3d::Identity();
    } else if (name == "nullspace_stiffness") {
      nullspace_stiffness_target_ = parameter.as_double();
    } else if (name == "nullspace_damping") {
      nullspace_damping_target_ = parameter.as_double();
    } else if (name == "joint1_nullspace_stiffness") {
      joint1_nullspace_stiffness_target_ = parameter.as_double();
    } else if (name == "translational_Ki") {
      Ki_target_.topLeftCorner(3, 3) = parameter.as_double() * Matrix3d::Identity();
    } else if (name == "rotational_Ki") {
      Ki_target_.bottomRightCorner(3, 3) = parameter.as_double() * Matrix3d::Identity();
    } else if (name == "delta_tau_max_") {
      delta_tau_max_target_ = parameter.as_double();
    } else if (name == "filter_params") {
      filter_params_ = parameter.as_double();
    } else if (name == "translational_clip_x") {
      translational_clip_max_.x() = parameter.as_double();
    } else if (name == "translational_clip_neg_x") {
      translational_clip_min_.x() = -parameter.as_double();
    } else if (name == "translational_clip_y") {
      translational_clip_max_.y() = parameter.as_double();
    } else if (name == "translational_clip_neg_y") {
      translational_clip_min_.y() = -parameter.as_double();
    } else if (name == "translational_clip_z") {
      translational_clip_max_.z() = parameter.as_double();
    } else if (name == "translational_clip_neg_z") {
      translational_clip_min_.z() = -parameter.as_double();
    } else if (name == "rotational_clip_x") {
      rotational_clip_max_.x() = parameter.as_double();
    } else if (name == "rotational_clip_neg_x") {
      rotational_clip_min_.x() = -parameter.as_double();
    } else if (name == "rotational_clip_y") {
      rotational_clip_max_.y() = parameter.as_double();
    } else if (name == "rotational_clip_neg_y") {
      rotational_clip_min_.y() = -parameter.as_double();
    } else if (name == "rotational_clip_z") {
      rotational_clip_max_.z() = parameter.as_double();
    } else if (name == "rotational_clip_neg_z") {
      rotational_clip_min_.z() = -parameter.as_double();
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid parameter: %s", name.c_str());
      result.successful = false;
      result.reason = "Invalid parameter";
    }
  }

  return result;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  start_time_ = this->get_node()->now();
  desired = Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  position_d_ = Vector3d(desired.block<3,1>(0,3));
  orientation_d_ = Quaterniond(desired.block<3,3>(0,0));
  orientation_d_.normalize();  // Normalize the desired orientation
  q_d_nullspace_ = Vector7d(franka_robot_model_->getRobotState()->q.data());

  // Initialize parameters to target values
  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  nullspace_damping_ = nullspace_damping_target_;
  joint1_nullspace_stiffness_ = joint1_nullspace_stiffness_target_;
  Ki_ = Ki_target_;

  update_counter_ = 0;
  control_rate_divider_ = 1;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // Only update if the counter matches the control rate
  if (update_counter_ % control_rate_divider_ != 0) {
    ++update_counter_;  // Increment the counter and skip this cycle
    return controller_interface::return_type::OK;
  }

  // Reset counter after completing a cycle
  ++update_counter_;
  if (update_counter_ >= control_rate_divider_) {
    update_counter_ = 0;
  }

  // Parameter filtering
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + 
                        (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + 
                      (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + 
                        (1.0 - filter_params_) * nullspace_stiffness_;
  nullspace_damping_ = filter_params_ * nullspace_damping_target_ + 
                        (1.0 - filter_params_) * nullspace_damping_;
  joint1_nullspace_stiffness_ = filter_params_ * joint1_nullspace_stiffness_target_ + 
                               (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;
  delta_tau_max_ = filter_params_ * delta_tau_max_target_ + (1.0 - filter_params_) * delta_tau_max_;

  auto node = get_node();
  //  Output params
  // std::stringstream ss;
  // ss << cartesian_stiffness_;
  // RCLCPP_INFO(node->get_logger(), "cartesian stiffness: %s", ss.str().c_str());
  // ss.str("");

  Eigen::Map<const Matrix4d> current(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Eigen::Vector3d position(current.block<3,1>(0,3));
  Eigen::Quaterniond orientation(current.block<3,3>(0,0));
  Eigen::Map<const Matrix7d> inertia(franka_robot_model_->getMassMatrix().data());
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
  Eigen::Map<const Vector7d> dq(franka_robot_model_->getRobotState()->dq.data());
  Eigen::Map<const Vector7d> tau_J_d(franka_robot_model_->getRobotState()->tau_J_d.data());

 // Publish the full Jacobian as a Float64MultiArray message
  franka_msgs::msg::FrankaModel jacobian_msg;
  jacobian_msg.header.stamp = this->get_node()->now();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 7; ++j) {
      jacobian_msg.ee_zero_jacobian[i * 7 + j] = jacobian(i, j);
    }
  }

  jacobian_publisher_->publish(jacobian_msg);

  // Calculate and publish Cartesian speed
  Eigen::Matrix<double, 6, 1> cartesian_speed = jacobian * dq;
  geometry_msgs::msg::Twist cartesian_speed_msg;
  cartesian_speed_msg.linear.x = cartesian_speed(0);
  cartesian_speed_msg.linear.y = cartesian_speed(1);
  cartesian_speed_msg.linear.z = cartesian_speed(2);
  cartesian_speed_msg.angular.x = cartesian_speed(3);
  cartesian_speed_msg.angular.y = cartesian_speed(4);
  cartesian_speed_msg.angular.z = cartesian_speed(5);
  cartesian_speed_publisher_->publish(cartesian_speed_msg);

  // translation error
  error_.head(3) << position - position_d_;
  // translation error clipping
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_[i]), translational_clip_max_[i]);
  }
  // orientation error
  orientation.normalize();
  orientation_d_.normalize();
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  Eigen::AngleAxisd error_quaternion_aa(error_quaternion);
  error_.tail(3) << error_quaternion_aa.axis() * error_quaternion_aa.angle();
  // Rotational error clipping
  for (int i = 0; i < 3; i++) {
    error_(i+3) = std::min(std::max(error_(i+3), rotational_clip_min_[i]), rotational_clip_max_[i]);
  }

  error_i.head(3) << (error_i.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i.tail(3) << (error_i.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  // Check if any joint is close to its limits
  Eigen::VectorXd proximity(7);
  for (int i = 0; i < 7; ++i) {
    if (q[i] <= joint_lower_limits_[i]) {
      proximity[i] = -1.0;  // Joint is at the lower limit
    } else if (q[i] >= joint_upper_limits_[i]) {
      proximity[i] = 1.0;   // Joint is at the upper limit
    } else {
      proximity[i] = 0.0;   // Joint is within limits
    }
  }

  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  
  tau_task << jacobian.transpose() * 
                  (-cartesian_stiffness_* error_ - cartesian_damping_ *(jacobian * dq) - Ki_ * error_i);

  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;

  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * qe - nullspace_damping_ * dqe);
  // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
  //                     jacobian.transpose() * jacobian_transpose_pinv) *
  //                        (nullspace_stiffness_ * qe -
  //                         (2.0 * sqrt(nullspace_stiffness_)) * dqe);

  tau_d <<  tau_task + coriolis + tau_nullspace;

  // Adjust torques near joint limits
  for (int i = 0; i < 7; ++i) {
    if (proximity[i] == -1.0 && tau_d[i] < 0.0) {
      // Joint is at the lower limit and torque is pushing further negative
      tau_d[i] = 0.0;
    } else if (proximity[i] == 1.0 && tau_d[i] > 0.0) {
      // Joint is at the upper limit and torque is pushing further positive
      tau_d[i] = 0.0;
    }
  }

  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::equilibriumPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  position_d_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
  error_i.setZero();
  Eigen::Quaterniond last_orientation_d_(orientation_d_);
  orientation_d_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  if (last_orientation_d_.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation_d_.coeffs() << -orientation_d_.coeffs();
  }
  orientation_d_.normalize();
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_serl
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_serl::CartesianImpedanceController,
                       controller_interface::ControllerInterface)