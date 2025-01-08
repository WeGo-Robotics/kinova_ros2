import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(JointTrajectory, '/gen3_lite_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        # JointTrajectory 메시지 생성
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # JointTrajectoryPoint 메시지 생성
        point = JointTrajectoryPoint()
        point.positions = [-0.00021624868051439705,
                            -0.2818977319137588,
                            1.3157371333986287,
                            -0.002829342342100638,
                            -1.046814588631844,
                            0.0003549993733075625]
        point.time_from_start.sec = 20
        point.time_from_start.nanosec = 0

        msg.points.append(point)

        # 메시지 퍼블리시
        self.publisher.publish(msg)
        self.get_logger().info('Published JointTrajectory: {}'.format(msg))


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
