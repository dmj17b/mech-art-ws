import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from datetime import datetime
import requests

class ImageSaveNode(Node):  
    def __init__(self):
        super().__init__('image_save_node')

        self.subscription = self.create_subscription(String, 'dalle_image_url', self.listener_callback, 10)

        self.declare_parameter('image_save_path', '/app/src/images')
        self.declare_parameter('default_url', "https://raw.githubusercontent.com/JTylerBoylan/mech-art-ws/main/default_image.webp")

        self.image_save_path = self.get_parameter('image_save_path').value
        self.default_url = self.get_parameter('default_url').value

        self.get_logger().info(f'\n'
            f'\t\t---Image Save Node---\n'
            f'\t\t Image Save Path: {self.image_save_path}\n'
            f'\t\t Default URL: {self.default_url}\n'
            f'\t\t--------------------------\n'
        )

    def listener_callback(self, msg : String):
        if msg.data == self.default_url:
            self.get_logger().info(f'Skipping Default Image...')
            return

        self.get_logger().info(f'Saving Image...')
        self.save_image(msg.data)
        self.get_logger().info(f'Saved Image.')

    def save_image(self, image_url: str):
        current_time = datetime.now().isoformat()
        filename = f"{self.image_save_path}/{current_time}.png"
        response = requests.get(image_url)
        if response.status_code == 200:
            with open(filename, 'wb') as file:
                file.write(response.content)
            print(f"Image saved as {filename}")
        else:
            print(f"Failed to retrieve the image. Status code: {response.status_code}")

def main(args=None):
    rclpy.init(args=args)
    image_save_node = ImageSaveNode()
    rclpy.spin(image_save_node)
    image_save_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()