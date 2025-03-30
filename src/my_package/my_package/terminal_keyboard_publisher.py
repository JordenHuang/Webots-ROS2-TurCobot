import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from the_interfaces.msg import TermKeyboard

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TermKeyboard, 'term_keyboard', 1)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def to_publish(self, key):
        msg = TermKeyboard()
        msg.key = key
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Publishing: '{msg.key}'")

    # def timer_callback(self):
    #     key = getch(timeout=0.1)
    #     if key == -1: key = ''
    #     msg.key = key
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f"Publishing: '{msg.key}'")

def getch_one():
    import sys, select, tty, termios, time
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rlist, _, _ = select.select([fd], [], [])
        c = -1  # c would be -1 or char that was read within timeout
        while c == -1:
            if fd in rlist:
                c = sys.stdin.read(1)
                return c
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    print(f"Press 'q' to exit")
    try:
        while True:
            key = getch_one()
            if key == -1:
                rclpy.spin_once(minimal_publisher, timeout_sec=0.1)
                continue
            elif key == 'q':
                break
            minimal_publisher.to_publish(key.capitalize())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()