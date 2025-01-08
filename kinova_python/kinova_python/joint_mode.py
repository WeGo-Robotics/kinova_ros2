import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController

class ControllerSwitchNode(Node):
    def __init__(self):
        super().__init__('controller_switch_node')
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /controller_manager/switch_controller...')

    def switch_controllers(self):
        request = SwitchController.Request()
        request.activate_controllers = ['gen3_lite_joint_trajectory_controller']
        request.deactivate_controllers = ['twist_controller']
        request.strictness = 1
        request.activate_asap = True

        # 컨트롤러 전환 요청
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Controllers switched successfully')
        else:
            self.get_logger().error('Failed to switch controllers')

        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    controller_switch_node = ControllerSwitchNode()
    controller_switch_node.switch_controllers()
    rclpy.spin(controller_switch_node)

if __name__ == '__main__':
    main()
