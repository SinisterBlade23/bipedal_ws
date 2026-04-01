import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import termios
import tty


class height_control(Node):
    def __init__(self):
        super().__init__("height_control")
        self.publisher_ = self.create_publisher(String, 'input_key', 10)
        self.get_logger().info(" Press W to increase height and S to decrease height")

        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        while True:
            key = self.get_key()

            msg = String()

            if key == 'w' or key == 'W':
                msg.data = "UP"
            elif key == 's' or key == 'S':
                msg.data = "DOWN"
            elif key == 'q' or key == 'Q':
                break
            else:
                continue 

            self.publisher_.publish(msg)



def main(args= None):
    rclpy.init(args=args)
    node = height_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    