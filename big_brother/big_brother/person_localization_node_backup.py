import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, PointCloud

import cv2
from cv_bridge import CvBridge
import numpy as np

class PersonLocalizationNode(Node):
    
        def __init__(self):
            super().__init__('person_localization_node')

            self.declare_parameter('weight_path', '/app/src/yolov3.weights')
            self.declare_parameter('config_path', '/app/src/yolov3.cfg')
            self.declare_parameter('label_path', '/app/src/coco.names')
            self.declare_parameter('homography_matrix_path', '/app/src/homography_matrix.npy')

            self.weight_path = self.get_parameter('weight_path').value
            self.config_path = self.get_parameter('config_path').value
            
            self.net : cv2.dnn.Net = cv2.dnn.readNet(self.weight_path, self.config_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

            self.layer_names = self.net.getLayerNames()
            self.output_layer_indices = self.net.getUnconnectedOutLayers().flatten()
            self.output_layers = [self.layer_names[i - 1] for i in self.output_layer_indices]

            self.label_path = self.get_parameter('label_path').value
            with open(self.label_path, "r") as f:
                self.classes = [line.strip() for line in f.readlines()]
            self.person_class_id = self.classes.index("person")

            self.homography_matrix_path = self.get_parameter('homography_matrix_path').value
            self.homography_matrix = np.load(self.homography_matrix_path)

            self.subscription = self.create_subscription(Image, 'camera_feed', self.listener_callback, 10)
            self.publisher_detected = self.create_publisher(Image, 'detected_persons', 10)
            self.publisher = self.create_publisher(PointCloud, 'person_locations', 10)

            self.get_logger().info(f'\n'
                f'\t---Person Localization Node---\n'
                f'\t Weight Path: {self.weight_path}\n'
                f'\t Config Path: {self.config_path}\n'
                f'\t Label Path: {self.label_path}\n'
                f'\t----------------------\n'
            )
        
        def listener_callback(self, msg : Image):
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            boxes, indices = self.detect_people_in_image(cv_image)
            self.publish_detected_persons(cv_image, boxes, indices)
            room_locations = self.get_room_locations(boxes, indices)
            self.publish_point_cloud(msg.header, room_locations)

        def detect_people_in_image(self, cv_image):
            height, width, _ = cv_image.shape

            blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)
            
            # Information to display
            class_ids, confidences, boxes = [], [], []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if class_id == self.person_class_id and confidence > 0.5:
                        # Person detected
                        center_x, center_y, width, height = (detection[0:4] * np.array([cv_image.shape[1], cv_image.shape[0], cv_image.shape[1], cv_image.shape[0]])).astype('int')
                        x = int(center_x - width / 2)
                        y = int(center_y - height / 2)
                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Non-max suppression to avoid multiple boxes
            indices = cv2.dnn.NMSBoxes(boxes, confidences, float(0.5), float(0.4))

            if len(indices) > 0 and type(indices[0]) is not np.int32:
                indices = indices[0]

            return boxes, indices
        
        def publish_detected_persons(self, cv_image, boxes, indices):
            for i in indices:
                box = boxes[i]
                x, y, w, h = box
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Person', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            detected_persons = CvBridge().cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_detected.publish(detected_persons)

        def get_room_locations(self, boxes, indices):
            person_locations = []
            for i in indices:
                box = boxes[i]
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