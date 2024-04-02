import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from openai import OpenAI

class ImageGenerationNode(Node):

    def __init__(self):
        super().__init__('image_generation_node')

        self.publisher_ = self.create_publisher(String, 'dalle_image_url', 10)
        self.subscription = self.create_subscription(String, 'dalle_prompt', self.listener_callback, 10)

        self.openai = OpenAI()

        self.declare_parameter('model', 'dall-e-3')
        self.declare_parameter('size', '1792x1024')
        self.declare_parameter('quality', 'standard')

        self.model = self.get_parameter('model').value
        self.size = self.get_parameter('size').value
        self.quality = self.get_parameter('quality').value

        self.get_logger().info(f'Image Generation Node Initialized with Model: {self.model}, Size: {self.size}, Quality: {self.quality}')

    def listener_callback(self, msg : String):

        self.get_logger().info(f'Generating Image...')
        image_url = self.generate_image(msg.data)
        
        msg = String()
        msg.data = image_url
        self.publisher_.publish(msg)

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