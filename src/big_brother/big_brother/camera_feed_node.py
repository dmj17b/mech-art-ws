import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class CameraFeedNode(Node):
    def __init__(self):
        super().__init__('camera_feed_node')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)

        self.declare_parameter('camera_device', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)

        self.camera_device = self.get_parameter('camera_device').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fps = self.get_parameter('camera_fps').value

        self.cap = cv2.VideoCapture(self.camera_device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0/self.camera_fps, self.timer_callback)

        self.get_logger().info(f'\n'
            f'\t---Camera Feed Node---\n'
            f'\t Camera Device: {self.camera_device}\n'
            f'\t Camera Width: {self.camera_width}\n'
            f'\t Camera Height: {self.camera_height}\n'
            f'\t Camera FPS: {self.camera_fps}\n'
            f'\t----------------------\n'
        )

    def __del__(self):
        self.cap.release()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_feed_node = CameraFeedNode()
    rclpy.spin(camera_feed_node)
    camera_feed_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()