import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray

from scipy.io.wavfile import write
import numpy as np

class AudioPlayNode(Node):
            
    def __init__(self):
        super().__init__('audio_play_node')

        self.subscription = self.create_subscription(Int16MultiArray, 'audio_data', self.listener_callback, 10)

        self.get_logger().info(f'\n'
            f'\t---Audio Play Node---\n'
            f'\t----------------------\n'
        )

    def listener_callback(self, msg : Int16MultiArray):
        audio_wav, sample_rate = np.array(msg.data, dtype=np.int16).reshape(-1, 1), msg.layout.data_offset
        # TODO

def main(args=None):
    rclpy.init(args=args)
    audio_play_node = AudioPlayNode()
    rclpy.spin(audio_play_node)
    audio_play_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()