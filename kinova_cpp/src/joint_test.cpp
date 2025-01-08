#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher")
    {
        // 퍼블리셔 생성
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/gen3_lite_joint_trajectory_controller/joint_trajectory", 10);

        // 1초마다 publish_trajectory 메서드를 호출하는 타이머 설정
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&JointTrajectoryPublisher::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        // JointTrajectory 메시지 생성
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.header.stamp = this->get_clock()->now();
        msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        // JointTrajectoryPoint 메시지 생성
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.time_from_start.sec = 2;
        point.time_from_start.nanosec = 0;

        msg.points.push_back(point);

        // 메시지 퍼블리시
        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published JointTrajectory: %s", msg.to_string().c_str());
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();

    // ROS2 노드를 실행하여 메시지를 퍼블리시합니다.
    rclcpp::spin(node);

    // 노드 종료
    rclcpp::shutdown();
    return 0;
}
