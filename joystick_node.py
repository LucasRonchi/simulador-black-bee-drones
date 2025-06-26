import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pynput import keyboard


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.joystick_input = [0.0, 0.0, 0.0, 0.0]  # [vertical, rotate, forward, lateral]

        # Create a subscriber to receive drone states
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_input', 1)


    def send_joystick_input(self):
        msg = Float32MultiArray()
        msg.data = self.joystick_input
        self.publisher_.publish(msg)


    def update_joystick_input_by_key(self, key_press):
        joystick_input = [0.0, 0.0, 0.0, 0.0]  # [vertical, rotate, forward, lateral]

        if key_press['w']:
            joystick_input[0] += 1.0
        if key_press['s']:
            joystick_input[0] -= 1.0
        
        if key_press['a']:
            joystick_input[1] -= 1.0
        if key_press['d']:
            joystick_input[1] += 1.0
        
        if key_press['up']:
            joystick_input[2] += 1.0
        if key_press['down']:
            joystick_input[2] -= 1.0

        if key_press['left']:
            joystick_input[3] -= 1.0
        if key_press['right']:
            joystick_input[3] += 1.0

        if self.joystick_input == joystick_input:
            return

        self.joystick_input = joystick_input
        self.send_joystick_input()


def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()

    key_press = {
        'w': False,
        'a': False,
        's': False,
        'd': False,
        'up': False,
        'left': False,
        'down': False,
        'right': False
    }


    def on_press(key, injected):
        nonlocal joystick_node, key_press

        if 'char' in dir(key):
            key_press[key.char] = True
    
        elif 'name' in dir(key):
            key_press[key.name] = True

            if key == keyboard.Key.esc:
                # Stop listener
                return False

        joystick_node.update_joystick_input_by_key(key_press)


    def on_release(key, injected):
        nonlocal joystick_node, key_press

        if 'char' in dir(key):
            key_press[key.char] = False

        elif 'name' in dir(key):
            key_press[key.name] = False

        joystick_node.update_joystick_input_by_key(key_press)


    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()

    joystick_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
