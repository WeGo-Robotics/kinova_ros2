#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TwistPublisherNode : public rclcpp::Node
{
public:
    TwistPublisherNode()
        : Node("twist_publisher_node")
    {
        // 파라미터 선언 및 가져오기
        topic_name_ = this->declare_parameter<std::string>("topic_name", "/twist_controller/commands");

        // 퍼블리셔 생성
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);

        // 0.1초마다 publish_twist 메서드를 호출하는 타이머 설정
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TwistPublisherNode::publish_twist, this));
    }

private:
    void publish_twist()
    {
        // Twist 메시지 생성
        auto twist_msg = geometry_msgs::msg::Twist();

        // Linear 속도 설정 (x, y, z)
        twist_msg.linear.x = 0.0;  // 전진 속도 (m/s)
        twist_msg.linear.y = 0.005;  // 좌/우 이동 속도 (m/s)
        twist_msg.linear.z = 0.0;  // 상/하 이동 속도 (m/s)

        // Angular 속도 설정 (x, y, z)
        twist_msg.angular.x = 0.0;  // Roll 회전 속도 (rad/s)
        twist_msg.angular.y = 0.0;  // Pitch 회전 속도 (rad/s)
        twist_msg.angular.z = 1.0;  // Yaw 회전 속도 (rad/s)

        // 메시지 퍼블리시
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear(x=%f, y=%f, z=%f), angular(x=%f, y=%f, z=%f)",
                    twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                    twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);
    }

    std::string topic_name_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto twist_publisher_node = std::make_shared<TwistPublisherNode>();

    // ROS2 노드를 실행하여 메시지를 퍼블리시합니다.
    rclcpp::spin(twist_publisher_node);

    // 노드 종료
    rclcpp::shutdown();
    return 0;
}
