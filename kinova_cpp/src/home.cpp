#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher"), goal_sent(false)
    {
        // 퍼블리셔 생성
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_lite_joint_trajectory_controller/joint_trajectory", 10);

        // 타이머 생성
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
        point.positions = {-0.00021624868051439705,
                            -0.2818977319137588,
                            1.3157371333986287,
                            -0.002829342342100638,
                            -1.046814588631844,
                            0.0003549993733075625};

        // 시간 설정
        int time = 30;
        point.time_from_start.sec = time;
        point.time_from_start.nanosec = 0;

        msg.points.push_back(point);

        // 메시지 퍼블리시
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published JointTrajectory");

        // 목표 동작이 이미 전송되었는지 확인하는 플래그 설정
        goal_sent = true;

        // 타이머 중지
        timer_->cancel();

        // 10초 후 노드 종료 예약
        auto shutdown_timer = this->create_wall_timer(
            std::chrono::seconds(time + 1),
            std::bind(&JointTrajectoryPublisher::shutdown_node, this));
    }

    void shutdown_node()
    {
        RCLCPP_INFO(this->get_logger(), "Motion completed. Shutting down node...");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_sent;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
