import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisherNode(Node):
    def __init__(self):
        super().__init__('twist_publisher_node')

        # 'topic_name' 파라미터 선언
        topic_name = self.declare_parameter('topic_name', '/twist_controller/commands').value

        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        
        # 0.1초마다 publish_twist 메서드를 호출하는 타이머 설정
        self.timer = self.create_timer(0.1, self.publish_twist)

    def publish_twist(self):
        # Twist 메시지 생성
        twist_msg = Twist()

        # Linear 속도 설정 (x, y, z)
        twist_msg.linear.x = 0.0  # 전진 속도 (m/s)
        twist_msg.linear.y = 0.005  # 좌/우 이동 속도 (m/s)
        twist_msg.linear.z = 0.0  # 상/하 이동 속도 (m/s)

        # Angular 속도 설정 (x, y, z)
        twist_msg.angular.x = 0.0  # Roll 회전 속도 (rad/s)
        twist_msg.angular.y = 0.0  # Pitch 회전 속도 (rad/s)
        twist_msg.angular.z = 1.0  # Yaw 회전 속도 (rad/s)

        # 메시지를 퍼블리시합니다.
        self.publisher.publish(twist_msg)
        self.get_logger().info(f'Publishing: linear(x={twist_msg.linear.x}, y={twist_msg.linear.y}, z={twist_msg.linear.z}), '
                               f'angular(x={twist_msg.angular.x}, y={twist_msg.angular.y}, z={twist_msg.angular.z})')

def main(args=None):
    rclpy.init(args=args)

    # 커맨드라인 인자로 받은 파라미터를 바탕으로 노드를 시작합니다.
    twist_publisher_node = TwistPublisherNode()

    # ROS2 노드를 실행하여 메시지를 퍼블리시합니다.
    rclpy.spin(twist_publisher_node)

    # 노드 종료
    twist_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
