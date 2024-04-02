import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray

import sounddevice as sd

class AudioRecordingNode(Node):
            
    def __init__(self):
        super().__init__('audio_recording_node')

        self.publisher = self.create_publisher(Int16MultiArray, 'audio_data', 10)
        self.subscription = self.create_subscription(Empty, 'record_audio', self.listener_callback, 10)

        self.declare_parameter('duration', 10)
        self.declare_parameter('sample_rate', 44100)

        self.duration = self.get_parameter('duration').value
        self.sample_rate = self.get_parameter('sample_rate').value

        self.get_logger().info(f'Audio Recording Node Initialized with Duration: {self.duration} and Sample Rate: {self.sample_rate}')

    def listener_callback(self, _):
        self.get_logger().info(f'Recording Audio...')
        audio_data = self.record_audio()
        self.publish_audio_data(audio_data)
        self.get_logger().info(f'Recorded Audio Data')

    def record_audio(self):
        audio_data = sd.rec(int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=2, dtype='int16')
        sd.wait()
        return audio_data

    def publish_audio_data(self, audio_data):
        msg = Int16MultiArray()
        msg.data = audio_data.flatten()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    audio_recording_node = AudioRecordingNode()
    rclpy.spin(audio_recording_node)
    audio_recording_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()