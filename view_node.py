import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class ViewNode(Node):
    def __init__(self):
        super().__init__('view_node')

        # Create a subscriber to receive drone states
        self.subscriber = self.create_subscription(
            String,
            'drone_states',
            self.drone_state_callback,
            1
        )


    def drone_state_callback(self, msg):
        try:
            states = json.loads(msg.data)
        except json.JSONDecodeError as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    view_node = ViewNode()
    rclpy.spin(view_node)
    view_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
