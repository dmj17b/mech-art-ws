import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

import cv2
import numpy as np

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')

        self.declare_parameter('calibration_width', 4.0)
        self.declare_parameter('calibration_height', 4.0)
        self.declare_parameter('save_file', '/app/src/homography_matrix.npy')

        self.calibration_width = self.get_parameter('calibration_width').value
        self.calibration_height = self.get_parameter('calibration_height').value
        self.save_file = self.get_parameter('save_file').value

        cv2.namedWindow("Video")
        cv2.setMouseCallback("Video", self.click_event)

        self.get_logger().info(f'\n'
            f'\t---Camera Calibration Node---\n'
            f'\t Calibration Width: {self.calibration_width}\n'
            f'\t Calibration Height: {self.calibration_height}\n'
            f'\t Save File: {self.save_file}\n'
            f'\t Click on the video in the following order:\n'
            f'\t  1. Room coordinates ({-self.calibration_width/2}, {-self.calibration_height/2})\n'
            f'\t  2. Room coordinates ({-self.calibration_width/2}, +{self.calibration_height/2})\n'
            f'\t  3. Room coordinates (+{self.calibration_width/2}, +{self.calibration_height/2})\n'
            f'\t  4. Room coordinates (+{self.calibration_width/2}, {-self.calibration_height/2})\n'
            f'\t----------------------\n'
        )

        cap = cv2.VideoCapture(0)
        self.calibration_points = []
        while True:
            ret, frame = cap.read()
            if not ret:
                self.get_logger().info("Failed to grab frame")
                break
            self.annotate_points(frame)
            cv2.imshow("Video", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or len(self.calibration_points) == 4:
                break

        cv2.destroyAllWindows()
        cap.release()

        real_world_corners = np.array([[-self.calibration_width/2, -self.calibration_height/2],
                                       [-self.calibration_width/2, +self.calibration_height/2],
                                       [+self.calibration_width/2, +self.calibration_height/2],
                                       [+self.calibration_width/2, -self.calibration_height/2]], dtype=np.float32)
        calibration_array = np.array(self.calibration_points, dtype=np.float32)
        homography_matrix, _ = cv2.findHomography(calibration_array, real_world_corners)
        np.save(self.save_file, homography_matrix)

        self.get_logger().info(f"Saved homography matrix to {self.save_file}")
        self.get_logger().info("Calibration complete.")

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.calibration_points.append((x, y))

    def annotate_points(self, frame):
        for idx, (x, y) in enumerate(self.calibration_points):
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f"{idx + 1}", (x, y), font, 0.5, (0, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    camera_calibration_node = CameraCalibrationNode()
    camera_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()