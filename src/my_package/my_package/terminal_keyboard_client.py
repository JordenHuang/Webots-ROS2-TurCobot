'''
Service experience
嘗試用service來做terminal的按鍵控制,但想錯了, 應該要用topic(一開始想對的,後來想錯了...)
但這個就留着當作歷程或是之後的參考
'''

import sys
import rclpy
from rclpy.node import Node
from the_interfaces.srv import TermKeyboard
from .util import getch


class TerminalKeyboardClient(Node):
    def __init__(self):
        super().__init__('terminal_keyboard_client')
        self.cli = self.create_client(TermKeyboard, 'term_keyboard')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TermKeyboard.Request()

    def send_request(self, key):
        self.req.key = key
        return self.cli.call_async(self.req)

def main():
    rclpy.init()

    tk_client = TerminalKeyboardClient()
    print(f"Press 'q' to exit")
    while True:
        key = getch(timeout=0.1)

        if key == -1: continue
        elif key == 'q': break
        future = tk_client.send_request(key.capitalize())
        rclpy.spin_until_future_complete(tk_client, future)

    tk_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()