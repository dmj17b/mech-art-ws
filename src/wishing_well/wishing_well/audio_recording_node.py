import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray

import sounddevice as sd
import numpy as np

class AudioRecordingNode(Node):
            
    def __init__(self):
        super().__init__('audio_recording_node')

        self.publisher = self.create_publisher(Int16MultiArray, 'audio_data', 10)
        self.subscription = self.create_subscription(Empty, 'record_audio', self.listener_callback, 10)

        self.declare_parameter('duration', 10)
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('device_id', 5)

        self.duration = self.get_parameter('duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_id = self.get_parameter('device_id').value

        device_info = sd.query_devices(self.device_id, 'input')
        self.get_logger().info(f'\n'
            f'\t---Audio Recording Node---\n'
            f'\t Duration: {self.duration}\n'
            f'\t Sample Rate: {self.sample_rate}\n'
            f'\t Device ID: {self.device_id}\n'
            f'\t Device Name: {device_info["name"]}\n'
            f'\t Channels: {device_info["max_input_channels"]}\n'
            f'\t Default Sample Rate: {device_info["default_samplerate"]}\n'
            f'\t--------------------------\n'
        )

    def listener_callback(self, _):
        self.get_logger().info(f'Recording Audio...')
        audio_data = self.record_audio()
        self.publish_audio_data(audio_data)
        self.get_logger().info(f'Recorded Audio Data')

    def record_audio(self):
        audio_data = sd.rec(int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='int16', device=self.device_id)
        sd.wait()
        return audio_data

    def publish_audio_data(self, audio_data : np.ndarray):
        audio_data_int16 = audio_data.astype(np.int16)
        flattened_audio_data = audio_data_int16.flatten()
        audio_data_list = flattened_audio_data.tolist()

        msg = Int16MultiArray()
        msg.data = audio_data_list
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    audio_recording_node = AudioRecordingNode()
    rclpy.spin(audio_recording_node)
    audio_recording_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()