import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(JointTrajectory, '/gen3_lite_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.goal_sent = False  # 목표 동작이 이미 전송되었는지 확인하는 플래그

    def publish_trajectory(self):
        # JointTrajectory 메시지 생성
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # JointTrajectoryPoint 메시지 생성
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        time = 30
        point.time_from_start.sec = time
        point.time_from_start.nanosec = 0

        msg.points.append(point)

        # 메시지 퍼블리시
        self.publisher.publish(msg)
        self.get_logger().info('Published JointTrajectory: {}'.format(msg))

        # 목표 전송 상태 업데이트 및 타이머 해제
        self.goal_sent = True
        self.timer.cancel()  # 추가 호출을 방지하기 위해 타이머 중지

        # 10초 후 노드 종료 예약
        self.create_timer(time+1, self.shutdown_node)

    def shutdown_node(self):
        self.get_logger().info('Motion completed. Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
