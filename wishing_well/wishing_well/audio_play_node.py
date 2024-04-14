import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray

import numpy as np
import sounddevice as sd

class AudioPlayNode(Node):
            
    def __init__(self):
        super().__init__('audio_play_node')

        self.subscription = self.create_subscription(Int16MultiArray, 'audio_data', self.listener_callback, 10)

        self.declare_parameter('device_id', -1)
        self.device_id = self.get_parameter('device_id').value

        self.declare_parameter('sample_rate', 44100)
        self.sample_rate = self.get_parameter('sample_rate').value


        device_info = sd.query_devices(self.device_id, 'input')
        self.get_logger().info(f'\n'
            f'\t---Audio Play Node---\n'
            f'\t Device ID: {self.device_id}\n'
            f'\t Device Name: {device_info["name"]}\n'
            f'\t Channels: {device_info["max_input_channels"]}\n'
            f'\t Default Sample Rate: {device_info["default_samplerate"]}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, msg : Int16MultiArray):
        audio_wav, sample_rate = np.array(msg.data, dtype=np.int16).reshape(-1, 1), msg.layout.data_offset
        # TODO: Play audio
        # sd.play(audio_wav, samplerate=sample_rate, channels=1)

def main(args=None):
    rclpy.init(args=args)
    audio_play_node = AudioPlayNode()
    rclpy.spin(audio_play_node)
    audio_play_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()