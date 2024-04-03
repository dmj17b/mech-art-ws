import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from openai import OpenAI

class WishExtractionNode(Node):
        
    def __init__(self):
        super().__init__('wish_extraction_node')

        self.publisher = self.create_publisher(String, 'wish_list', 10)
        self.subscription = self.create_subscription(String, 'transcript', self.listener_callback, 10)

        self.openai = OpenAI()

        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('prompt_file', '/app/gpt-audio-prompt.txt')

        self.model = self.get_parameter('model').value
        self.prompt_file = self.get_parameter('prompt_file').value

        with open(self.prompt_file, 'r') as file:
            self.gpt_audio_prompt = file.read()

        self.wish_list = []

        self.get_logger().info(f'\n'
            f'\t---Wish Extraction Node---\n'
            f'\t Model: {self.model}\n'
            f'\t Prompt File: {self.prompt_file}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, msg : String):
        self.get_logger().info(f'Extracting Wish...')
        wish = self.get_wishes_from_transcript(msg.data)
        self.parse_wish(wish)
        wish_list = self.format_wish_list()

        msg = String()
        msg.data = ",".join(wish_list)
        self.publisher.publish(msg)
        self.get_logger().info(f'Extracted Wish List: {wish_list}')

    def get_wishes_from_transcript(self, transcript: str):
        response = self.openai.chat.completions.create(
            model=self.model,
            messages=[
            {
                "role": "system",
                "content": self.gpt_audio_prompt
            },
            {
                "role": "user",
                "content": transcript
            }
            ],
            max_tokens=1000
        )
        return response.choices[0].message.content
    
    def parse_wish(self, wish: str):
        wish = wish.rstrip().lower().replace("\"", "")

        if (wish.find("none") != -1) or (len(wish) == 0):
            self.get_logger().info(f'Skipping empty wish...')
            return
        
        self.get_logger().info(f'Extracted Wish: {wish}')
        
        if (wish.find("reset") != -1):
            self.get_logger().info(f'Resetting wish list...')
            self.wish_list.clear()
        else:
            self.wish_list.append(wish)

    def format_wish_list(self):
        wish_list_reverse = self.wish_list.copy()
        wish_list_reverse.reverse()
        return wish_list_reverse

def main(args=None):
    rclpy.init(args=args)
    wish_extraction_node = WishExtractionNode()
    rclpy.spin(wish_extraction_node)
    wish_extraction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()