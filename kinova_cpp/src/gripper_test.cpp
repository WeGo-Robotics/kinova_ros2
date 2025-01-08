#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include <memory>
#include <iostream>

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

class GripperControlNode : public rclcpp::Node
{
public:
    GripperControlNode() : Node("gripper_control_node")
    {
        // 액션 클라이언트 생성
        _action_client = rclcpp_action::create_client<GripperCommand>(
            this, "/gen3_lite_2f_gripper_controller/gripper_cmd");

        // 액션 서버가 준비될 때까지 대기
        while (!_action_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
    }

    // 그리퍼 명령 보내기
    void send_gripper_command(float position, float max_effort = 10.0)
    {
        // 목표 메시지 설정 
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;  // 0.0 for open, 1.0 for close
        goal_msg.command.max_effort = max_effort;  // 최대 힘 설정
        RCLCPP_INFO(this->get_logger(), "Sending gripper command: position=%.2f, max_effort=%.2f", position, max_effort);

        // 목표를 액션 서버에 비동기적으로 전송
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GripperControlNode::result_callback, this, std::placeholders::_1);
        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    // 액션 클라이언트
    rclcpp_action::Client<GripperCommand>::SharedPtr _action_client;

    // 액션 결과 콜백
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Gripper command succeeded.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gripper_node = std::make_shared<GripperControlNode>();

    // 사용자 입력 받기
    while (rclcpp::ok()) {
        std::string user_input;
        std::cout << "Enter 'open' to open the gripper or 'close' to close it: ";
        std::getline(std::cin, user_input);

        if (user_input == "open") {
            gripper_node->send_gripper_command(0.5);  // 그리퍼 열기
        } else if (user_input == "close") {
            gripper_node->send_gripper_command(0.9);  // 그리퍼 닫기
        } else {
            std::cout << "Invalid input! Please enter 'open' or 'close'." << std::endl;
        }
    }

    rclcpp::spin(gripper_node);
    rclcpp::shutdown();
    return 0;
}
