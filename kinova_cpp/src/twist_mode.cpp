#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

class ControllerSwitchNode : public rclcpp::Node
{
public:
    ControllerSwitchNode() : Node("controller_switch_node")
    {
        // 클라이언트 생성
        client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        // 서비스가 준비될 때까지 기다리기
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service /controller_manager/switch_controller...");
        }
    }

    void switch_controllers()
    {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = {"twist_controller"};
        request->deactivate_controllers = {"gen3_lite_joint_trajectory_controller"};
        request->strictness = 1;
        request->activate_asap = true;

        // 비동기 호출
        auto future = client_->async_send_request(request);

        // 비동기 요청 완료 대기
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Controllers switched successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers");
        }

        // 노드 종료
        RCLCPP_INFO(this->get_logger(), "Shutting down node...");
        rclcpp::shutdown();
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerSwitchNode>();

    node->switch_controllers();

    rclcpp::spin(node); // 노드 실행

    return 0;
}
