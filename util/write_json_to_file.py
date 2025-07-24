"""Quick node to write JSON strings to a file. Did this to inspect the iperf output"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JSONLogger(Node):
    def __init__(self):
        super().__init__('json_logger')
        self.subscription = self.create_subscription(
            String,
            '/doodle_monitor/iperf_result',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        json_string = msg.data

        # Save to file (append mode)
        with open("output.json", "a") as f:
            f.write(json_string + "\n")  # Add newline if multiple entries

        self.get_logger().info("Wrote full JSON to file.")

def main(args=None):
    rclpy.init(args=args)
    node = JSONLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
