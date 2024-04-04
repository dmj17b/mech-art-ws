import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from openai import OpenAI

class ImageGenerationNode(Node):

    def __init__(self):
        super().__init__('image_generation_node')

        self.publisher = self.create_publisher(String, 'dalle_image_url', 10)
        self.subscription = self.create_subscription(String, 'dalle_prompt', self.listener_callback, 10)

        self.openai = OpenAI()

        self.declare_parameter('model', 'dall-e-3')
        self.declare_parameter('size', '1792x1024')
        self.declare_parameter('quality', 'standard')
        self.declare_parameter('default_url', "https://raw.githubusercontent.com/JTylerBoylan/mech-art-ws/main/default_image.webp")

        self.model = self.get_parameter('model').value
        self.size = self.get_parameter('size').value
        self.quality = self.get_parameter('quality').value
        self.default_url = self.get_parameter('default_url').value

        default_url_msg = String()
        default_url_msg.data = self.default_url
        self.publisher.publish(default_url_msg)

        self.get_logger().info(f'\n'
            f'\t---Image Generation Node---\n'
            f'\t Model: {self.model}\n'
            f'\t Size: {self.size}\n'
            f'\t Quality: {self.quality}\n'
            f'\t Default URL: {self.default_url}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, msg : String):

        if msg.data == '':
            self.get_logger().info(f'Empty Prompt Received. Using Default Image')
            image_url = self.default_url
        else:
            self.get_logger().info(f'Generating Image...')
            try:
                image_url = self.generate_image(msg.data)
            except Exception as e:
                self.get_logger().error(f'Error Generating Image: {e}')
                image_url = self.default_url
        
        msg = String()
        msg.data = image_url
        self.publisher.publish(msg)

        self.get_logger().info(f'Generated Image: {image_url}')

    def generate_image(self, prompt: str):
        return self.openai.images.generate(
            model=self.model,
            prompt=prompt,
            size=self.size,
            quality=self.quality,
            n=1,
        ).data[0].url
    
def main(args=None):
    rclpy.init(args=args)
    image_generator_node = ImageGenerationNode()
    rclpy.spin(image_generator_node)
    image_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()