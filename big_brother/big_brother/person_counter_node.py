import rclpy
from rclpy.node import Node

from std_msgs import Int32
from sensor_msgs import PointCloud

class PersonCounterNode(Node):
    def __init__(self):
        super().__init__('person_counter_node')

        self.subscription = self.create_subscription(PointCloud, 'person_locations', self.listener_callback, 10)
        self.publisher = self.create_publisher(Int32, 'person_count', 10)

        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius', 4.0)

        self.get_logger().info(f'\n'
            f'\t---Person Counter Node---\n'
            f'\tCenter X: {self.get_parameter("center_x").value}\n'
            f'\tCenter Y: {self.get_parameter("center_y").value}\n'
            f'\tRadius: {self.get_parameter("radius").value}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, msg : PointCloud):
        self.get_logger().info(f'Counting People...')
        count = 0
        for point in msg.points:
            x, y = point.x, point.y
            center_x = self.get_parameter('center_x').value
            center_y = self.get_parameter('center_y').value
            radius = self.get_parameter('radius').value
            if (x - center_x)**2 + (y - center_y)**2 < radius**2:
                count += 1
        self.publish_person_count(count)

    def publish_person_count(self, count: int):
        msg = Int32()
        msg.data = count
        self.publisher.publish(msg)
        self.get_logger().info(f'Counted {count} people.')