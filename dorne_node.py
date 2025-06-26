import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from drone import Drone
import json


class DorneNode(Node):
    def __init__(self):
        super().__init__('dorne_node')
        self.drone = Drone()

        # timer
        self.timer = self.create_timer(0.1, self.update_drone)

        # Create a subscriber to receive joystick input
        self.joystick_subscriber = self.create_subscription(
            Float32MultiArray,
            'joystick_input',
            self.joystick_callback,
            1
        )

        self.publisher_ = self.create_publisher(String, 'drone_states', 1)


    def send_drone_state(self):
        states = self.drone.get_states()
        states_json = json.dumps(states)

        msg = String()
        msg.data = states_json

        self.publisher_.publish(msg)


    def update_drone(self):
        self.drone.update()
        self.send_drone_state()


    def joystick_callback(self, msg):
        vertical, rotate, forward, lateral = msg.data
        self.drone.update_joystick_input(vertical, rotate, forward, lateral)
        self.send_drone_state()


def main(args=None):
    rclpy.init(args=args)
    drone_node = DorneNode()
    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
