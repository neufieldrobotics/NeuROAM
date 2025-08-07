import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Change this to match your message type
import time

CAMERA_FREQ = 20

class DownsamplerNode(Node):
    def __init__(self):
        super().__init__('downsampler_node')

        self.declare_parameter('sampling_freq', 5.0)  # Publish at max 1 Hz
        self.sampling_freq = self.get_parameter('sampling_freq').get_parameter_value().double_value

        # ADD CHECK FOR PERFECT FACTOR ONLY DOWNSAMPLES ??
        if self.sampling_freq > CAMERA_FREQ:
            self.get_logger().error(f"Downsampling rate at {self.sampling_freq}, is set  higher than original rate")
        else:
            self.skipcount = CAMERA_FREQ / self.sampling_freq

        self.cam0_input_topic = '/cam_sync/cam0/image_raw'
        self.cam1_input_topic = '/cam_sync/cam1/image_raw'

        self.cam0_output_topic = '/downsampled_imgs/cam0/image_raw'
        self.cam1_output_topic = '/downsampled_imgs/cam1/image_raw'

        self.skipcount_cam0 = 0
        self.skipcount_cam1 = 0
        self.cam0_subscriber = self.create_subscription(
            Image,
            self.cam0_input_topic,
            self.callback_cam0,
            10
        )

        self.cam1_subscriber = self.create_subscription(
            Image,
            self.cam1_input_topic,
            self.callback_cam1,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            self.cam0_output_topic,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            self.cam1_output_topic,
            10
        )

        self.get_logger().info(f"Subscribed to: {self.cam0_input_topic}")
        self.get_logger().info(f"Subscribed to: {self.cam1_input_topic}")
        self.get_logger().info(f"Publishing downsampled messages to: {self.cam0_output_topic} at {self.sampling_freq} Hz")
        self.get_logger().info(f"Publishing downsampled messages to: {self.cam1_output_topic} at {self.sampling_freq} Hz")

    def callback_cam0(self, msg):
        
        if self.skipcount_cam0 % self.sampling_freq == 0:
            self.publisher.publish(msg)
            self.skipcount_cam0 = 0
        else:
            self.skipcount_cam0 += 1

    def callback_cam1(self, msg):

        if self.skipcount_cam1 % self.sampling_freq == 0:
            self.publisher.publish(msg)
            self.skipcount_cam1 = 0
        else:
            self.skipcount_cam1 += 1

def main(args=None):
    rclpy.init(args=args)
    node = DownsamplerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()