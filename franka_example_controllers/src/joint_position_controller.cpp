/*
Refered to Source file:
  https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/joint_position_example_controller.cpp
*/

#include <franka_example_controllers/joint_position_controller.hpp>

#include <cmath>
#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("target_joint_positions", {0.0, -0.785, 0.0, -2.35, 0.0, 1.571, 0.785});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(const rclcpp_lifecycle::State& /*state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();

  // Get target joint positions
  
  auto target_positions = node->get_parameter("target_joint_positions").as_double_array();
  if (target_positions.size() != num_joints) {
    RCLCPP_ERROR(node->get_logger(), "JointPositionController: Incorrect number of target joint positions");
    return CallbackReturn::FAILURE;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    reset_pose_[i] = target_positions[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State& /*state*/) {
  updateJointStates();
  auto node = get_node();
  start_time_ = node->now();
  initial_q_ = q_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  auto elapsed_time = get_node()->now() - start_time_;
  double elapsed_seconds = elapsed_time.seconds();

  for (size_t i = 0; i < num_joints; ++i) {
    if (elapsed_seconds > 10) {
      command_interfaces_[i].set_value(reset_pose_[i]);
    } else {
      double interpolation = (elapsed_seconds / 10.0) * reset_pose_[i] + ((10 - elapsed_seconds) / 10.0) * initial_q_(i);
      command_interfaces_[i].set_value(interpolation);
    }
  }
  return controller_interface::return_type::OK;
}

void JointPositionController::updateJointStates() {
  for (size_t i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionController, controller_interface::ControllerInterface)
