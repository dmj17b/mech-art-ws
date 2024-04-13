import rclpy
from rclpy import Node

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

import numpy as np
import cv2

class PersonLocatorNode(Node):

        def __init__(self):
            super().__init__('person_locator_node')

            self.homography_matrix_path = self.get_parameter('homography_matrix_path').value
            self.homography_matrix = np.load(self.homography_matrix_path)

            self.subscription = self.create_subscription(Detection2DArray, 'detectnet/detections', self.listener_callback, 10)
            self.publisher = self.create_publisher(PointCloud, 'person_locations', 10)

            self.get_logger().info(f'\n'
                f'\t---Person Locator Node---\n'
                f'\tHomography Matrix Path: {self.homography_matrix_path}\n'
                f'\t----------------------\n'
            )

        def listener_callback(self, msg : Detection2DArray):
            room_locations = self.get_room_locations(msg)
            self.publish_point_cloud(msg.header, room_locations)

        def get_room_locations(self, msg : Detection2DArray):
            person_locations = []
            for detection in msg.detections:
                x = detection.bbox.center.x
                y = detection.bbox.center.y
                h = detection.bbox.size_y
                person_locations.append((x, y + h))

            person_locations = np.array(person_locations, dtype=np.float32).reshape(-1, 1, 2)
            room_locations = cv2.perspectiveTransform(person_locations, self.homography_matrix)
            return room_locations
        
        def publish_point_cloud(self, header, room_locations):
            point_cloud = PointCloud()
            point_cloud.header = header
            point_cloud.points = []
            if room_locations is not None:
                for room_location in room_locations:
                    x, y = room_location[0]
                    point = Point32()
                    point.x = float(x)
                    point.y = float(y)
                    point_cloud.points.append(point)
            self.publisher.publish(point_cloud)
        
def main(args=None):
    rclpy.init(args=args)
    person_locator_node = PersonLocatorNode()
    rclpy.spin(person_locator_node)
    person_locator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()