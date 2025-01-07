#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>

class FrankaGripperServer : public rclcpp::Node
{
public:
    using Move = franka_msgs::action::Move;
    using Grasp = franka_msgs::action::Grasp;
    using Homing = franka_msgs::action::Homing;

    FrankaGripperServer()
        : Node("franka_gripper_server"), current_position_(0.0348)
    {
        command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/franka/gripper", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FrankaGripperServer::_control_gripper, this, std::placeholders::_1));

        gripper_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/panda_gripper/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FrankaGripperServer::update_position, this, std::placeholders::_1));

        move_client_ = rclcpp_action::create_client<Move>(this, "panda_gripper/move");
        grasp_client_ = rclcpp_action::create_client<Grasp>(this, "panda_gripper/grasp");
        homing_client_ = rclcpp_action::create_client<Homing>(this, "panda_gripper/homing");

        prev_move_time_ = std::chrono::high_resolution_clock::now();
    }

    void open(double width, double speed = 0.5)
    {
        auto goal_msg = Move::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;
        move_client_->async_send_goal(goal_msg);
    }

    void close(double width = 0.0, double speed = 0.5)
    {
        auto goal_msg = Grasp::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;
        goal_msg.force = 130.0;
        goal_msg.epsilon.inner = 1.0;
        goal_msg.epsilon.outer = 1.0;
        grasp_client_->async_send_goal(goal_msg);
    }

    void homing()
    {
        auto goal_msg = Homing::Goal();
        homing_client_->async_send_goal(goal_msg);
    }

private:
    void update_position(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_position_ = msg->position[0]*2;
    }

    void _control_gripper(const std_msgs::msg::Float32::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), "Current position: %.4f", current_position_);
        RCLCPP_INFO(this->get_logger(), "Commanded position: %.4f", msg->data);
        std::cout << "current_position_: " << current_position_ << std::endl;
        std::cout << "msg->data: " << msg->data << std::endl;
        current_time_ = std::chrono::high_resolution_clock::now();
        if (current_time_ - prev_move_time_ > std::chrono::milliseconds(500))
        {
            if (msg->data >= 2.0)
            {
                homing();
                prev_move_time_ = current_time_;
            }
            else if (msg->data > current_position_)
            {
                open(msg->data);
                prev_move_time_ = current_time_;
            }
            else if (msg->data < current_position_)
            {
                close();
                prev_move_time_ = current_time_;
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_states_sub_;
    rclcpp_action::Client<Move>::SharedPtr move_client_;
    rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
    rclcpp_action::Client<Homing>::SharedPtr homing_client_;
    std::string current_action_;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_move_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> current_time_;
    double current_position_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaGripperServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}