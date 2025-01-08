import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rclpy.node import Node

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self._action_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')

    def send_gripper_command(self, position: float, max_effort: float = 10.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # 0.0 for open, 1.0 for close
        goal_msg.command.max_effort = max_effort  # Set max effort for the gripper
        self.get_logger().info(f'Sending gripper command: position={position}, max_effort={max_effort}')
        
        # Send the goal to the action server
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperControlNode()

    while rclpy.ok():
        user_input = input("Enter 'open' to open the gripper or 'close' to close it: ").strip().lower()

        if user_input == 'open':
            gripper_node.send_gripper_command(0.0)  # Open the gripper
        elif user_input == 'close':
            gripper_node.send_gripper_command(1.0)  # Close the gripper
        else:
            print("Invalid input! Please enter 'open' or 'close'.")

    rclpy.spin(gripper_node)

if __name__ == '__main__':
    main()
