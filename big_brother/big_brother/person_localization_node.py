import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, PointCloud

import cv2
from cv_bridge import CvBridge
import numpy as np

from ultralytics import YOLO

class PersonLocalizationNode(Node):
    
        def __init__(self):
            super().__init__('person_localization_node')

            self.declare_parameter('model', 'yolov8n')
            self.declare_parameter('homography_matrix_path', '/app/src/homography_matrix.npy')

            self.model_name = self.get_parameter('model').value

            self.model = YOLO(self.model_name + '.pt')

            self.homography_matrix_path = self.get_parameter('homography_matrix_path').value
            self.homography_matrix = np.load(self.homography_matrix_path)

            self.subscription = self.create_subscription(Image, 'camera_feed', self.listener_callback, 10)
            self.publisher_detected = self.create_publisher(Image, 'detected_persons', 10)
            self.publisher = self.create_publisher(PointCloud, 'person_locations', 10)

            self.get_logger().info(f'\n'
                f'\t---Person Localization Node---\n'
                f'\tModel: {self.model_name}\n'
                f'\tHomography Matrix Path: {self.homography_matrix_path}\n'
                f'\t----------------------\n'
            )
        
        def listener_callback(self, msg : Image):
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            boxes = self.detect_people_in_image(cv_image)
            self.publish_detected_persons(cv_image, boxes)
            room_locations = self.get_room_locations(boxes)
            self.publish_point_cloud(msg.header, room_locations)

        def detect_people_in_image(self, cv_image):
            results = self.model(cv_image)[0]
            boxes = results.boxes
            
            bboxes = []
            for i in range(len(boxes.cls)):
                cls = boxes.cls[i]
                xywh = boxes.xywh[i]
                x, y, w, h = xywh[0], xywh[1], xywh[2], xywh[3]
                if cls == 0:
                    bboxes.append((int(x - w/2), int(y - h/2), int(w), int(h)))
                
            return bboxes

        
        def publish_detected_persons(self, cv_image, boxes):
            for box in boxes:
                x, y, w, h = box
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Person', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            detected_persons = CvBridge().cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_detected.publish(detected_persons)

        def get_room_locations(self, boxes):
            person_locations = []
            for box in boxes:
                x, y, w, h = box
                person_locations.append((x + w // 2, y + h))

            # Apply perspective transform to get the room locations
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
    person_localization_node = PersonLocalizationNode()
    rclpy.spin(person_localization_node)
    person_localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()