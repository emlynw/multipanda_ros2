#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
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
        : Node("franka_gripper_server"), current_action_(""), prev_width_(0.09)
    {
        gripper_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/franka/gripper", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FrankaGripperServer::_control_gripper, this, std::placeholders::_1));

        move_client_ = rclcpp_action::create_client<Move>(this, "panda_gripper/move");
        grasp_client_ = rclcpp_action::create_client<Grasp>(this, "panda_gripper/grasp");
        homing_client_ = rclcpp_action::create_client<Homing>(this, "panda_gripper/homing");

        prev_move_time_ = std::chrono::high_resolution_clock::now();
    }

    void open(double width, double speed = 0.3)
    {
        if (width == prev_width_)
            return;

        current_action_ = "open";
        auto goal_msg = Move::Goal();
        goal_msg.width = 0.08-0.06745*(width+1);
        goal_msg.speed = speed;
        move_client_->async_send_goal(goal_msg);
        prev_width_ = width;
    }

    void close(double width = 0.01, double speed = 0.3)
    {
        if (current_action_ == "close")
            return;

        current_action_ = "close";
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
        if (current_action_ == "homing")
            return;

        current_action_ = "homing";
        auto goal_msg = Homing::Goal();
        homing_client_->async_send_goal(goal_msg);
    }

private:
    void _control_gripper(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // if 500ms has passed since the last move command, send a new one
        current_time_ = std::chrono::high_resolution_clock::now();
        if (current_time_ - prev_move_time_ > std::chrono::milliseconds(500))
        {
            if (msg->data > 0)
            {
                close();
            }
            else
            {
                open(msg->data);
            }
            prev_move_time_ = current_time_;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
    rclcpp_action::Client<Move>::SharedPtr move_client_;
    rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
    rclcpp_action::Client<Homing>::SharedPtr homing_client_;
    std::string current_action_;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_move_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> current_time_;
    double prev_width_; 

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaGripperServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}