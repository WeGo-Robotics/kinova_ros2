#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

class ControllerSwitchNode : public rclcpp::Node
{
public:
    ControllerSwitchNode()
        : Node("controller_switch_node")
    {
        // SwitchController 서비스 클라이언트 생성
        client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        // 서비스가 활성화될 때까지 기다리기
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service /controller_manager/switch_controller...");
        }
    }

    void switch_controllers()
    {
        // SwitchController 서비스 요청 생성
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = {"gen3_lite_joint_trajectory_controller"};
        request->deactivate_controllers = {"twist_controller"};
        request->strictness = 1;
        request->activate_asap = true;

        // 서비스 호출
        auto future = client_->async_send_request(request);

        // 서비스 결과 기다리기
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Controllers switched successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers");
        }

        // 노드 종료
        RCLCPP_INFO(this->get_logger(), "Shutting down node...");
        rclcpp::shutdown(); // shutdown을 여기서 호출
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_switch_node = std::make_shared<ControllerSwitchNode>();
    controller_switch_node->switch_controllers();
    // spin 호출 제거
    return 0;
}
