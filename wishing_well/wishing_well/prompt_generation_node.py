import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from openai import OpenAI

class PromptGenerationNode(Node):
    
    def __init__(self):
        super().__init__('prompt_generation_node')

        self.publisher = self.create_publisher(String, 'dalle_prompt', 10)
        self.subscription = self.create_subscription(String, 'wish_list', self.listener_callback, 10)

        self.openai = OpenAI()

        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('prompt_file', '/app/src/gpt-image-prompt.txt')

        self.model = self.get_parameter('model').value
        self.prompt_file = self.get_parameter('prompt_file').value

        with open(self.prompt_file, 'r') as file:
            self.gpt_image_prompt = file.read()

        self.get_logger().info(f'\n'
            f'\t---Prompt Generation Node---\n'
            f'\t Model: {self.model}\n'
            f'\t Prompt File: {self.prompt_file}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, wish_list_msg : String):
        prompt_msg = String()
        if wish_list_msg.data == "":
            self.get_logger().info(f'Empty Wish List, Skipping Prompt Generation')
            prompt_msg.data = ""
            self.publisher.publish(prompt_msg)
            return
        else:
            self.get_logger().info(f'Generating Prompt...')
            dalle_prompt = self.generate_dalle_prompt(wish_list_msg.data)
            prompt_msg.data = dalle_prompt
            self.publisher.publish(prompt_msg)
            self.get_logger().info(f'Generated Prompt: {dalle_prompt}')

    def generate_dalle_prompt(self, wish_list : str):
        response = self.openai.chat.completions.create(
            model=self.model,
            messages=[
            {
                "role": "system",
                "content": self.gpt_image_prompt
            },
            {
                "role": "user",
                "content": wish_list
            }
            ],
            max_tokens=1000
        )
        return response.choices[0].message.content
        
def main(args=None):
    rclpy.init(args=args)
    prompt_generation_node = PromptGenerationNode()
    rclpy.spin(prompt_generation_node)
    prompt_generation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()