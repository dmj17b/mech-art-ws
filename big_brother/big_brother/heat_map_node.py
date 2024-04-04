import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from cv_bridge import CvBridge

import cv2
import numpy as np

class HeatmapNode(Node):

    def __init__(self):
        super().__init__('heatmap_node')
        
        self.declare_parameter('x_width', 16)
        self.declare_parameter('y_height', 20)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.x_width = self.get_parameter('x_width').value
        self.y_height = self.get_parameter('y_height').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        self.subscription = self.create_subscription(PointCloud, 'person_locations', self.listener_callback, 10)

        self.publisher = self.create_publisher(Image, 'heat_map', 10)

    def listener_callback(self, msg : PointCloud):
        heat_map = self.create_heat_map(msg)
        self.publisher.publish(heat_map)

    def create_heat_map(self, msg : PointCloud):
        heat_map = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        heat_map[:,:] = [127, 0, 0]
        for point in msg.points:
            x = point.x
            y = point.y
            x = x / self.x_width + 0.5
            y = y / self.y_height + 0.5
            y = -y
            cv2.circle(heat_map, (int(x * self.image_width), int(y * self.image_height)), 10, (0, 0, 255), -1)
        heat_map = cv2.GaussianBlur(heat_map, (15, 15), 0)
        return CvBridge().cv2_to_imgmsg(heat_map, encoding='bgr8')
    
def main(args=None):
    rclpy.init(args=args)
    heat_map_node = HeatmapNode()
    rclpy.spin(heat_map_node)
    heat_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()