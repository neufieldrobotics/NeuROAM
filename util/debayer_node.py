import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage  # Change this to match your message type
from cv_bridge import CvBridge
import cv2
import numpy as np

CAMERA_FREQ = 20

class CameraParamsNode(Node):
    def __init__(self):
        super().__init__('cameraparams_node')

        self.cam0_input_topic = '/cam_sync/cam0/image_raw/debayered'
        self.cam1_input_topic = '/cam_sync/cam1/image_raw/debayered'
        self.cam0_input_topic_compressed = '/cam_sync/cam0/image_raw/compressed'
        self.cam1_input_topic_compressed = '/cam_sync/cam1/image_raw/compressed'

        self.cam0_subscriber_compressed = self.create_subscription(
            CompressedImage,
            self.cam0_input_topic_compressed,
            self.callback_cam0_compressed,
            10
        )

        self.cam1_subscriber_compressed = self.create_subscription(
            CompressedImage,
            self.cam1_input_topic_compressed,
            self.callback_cam1_compressed,
            10
        )

        self.bridge = CvBridge()

        self.cam0_publisher = self.create_publisher(Image, self.cam0_input_topic, 10) 
        self.cam1_publisher = self.create_publisher(Image, self.cam1_input_topic, 10) 

    def callback_cam0_compressed(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        bayer_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        rgb_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_RG2BGR)

        image_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
        image_msg.header = msg.header  # Preserve original timestamp/frame_id
        self.cam0_publisher.publish(image_msg)
        
    def callback_cam1_compressed(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        bayer_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        rgb_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_RG2RGB)

        image_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
        image_msg.header = msg.header  # Preserve original timestamp/frame_id
        self.cam1_publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
