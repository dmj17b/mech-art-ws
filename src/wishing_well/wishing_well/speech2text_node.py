import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from openai import OpenAI

from scipy.io.wavfile import write
import numpy as np

class Speech2TextNode(Node):
        
    def __init__(self):
        super().__init__('speech2text_node')

        self.publisher = self.create_publisher(String, 'transcript', 10)
        self.subscription = self.create_subscription(Int16MultiArray, 'audio_data', self.listener_callback, 10)

        self.declare_parameter('model', 'whisper-1')
        self.declare_parameter('sample_rate', 44100)

        self.model = self.get_parameter('model').value
        self.sample_rate = self.get_parameter('sample_rate').value

        self.openai = OpenAI()

        self.get_logger().info(f'\n'
            f'\t\t---Speech2Text Node---\n'
            f'\t\t Model: {self.model}\n'
            f'\t\t Sample Rate: {self.sample_rate}\n'
            f'\t\t----------------------\n'
        )

    def listener_callback(self, audio_data_msg : Int16MultiArray):
        self.get_logger().info(f'Converting Audio to Text...')
        transcript = self.get_transcript_from_audio(audio_data_msg.data)
        
        transcript_msg = String()
        transcript_msg.data = transcript
        self.publisher.publish(transcript_msg)
        self.get_logger().info(f'Converted Audio to Text: {transcript}')
        
    def get_transcript_from_audio(self, recording):
        recording_array = np.array(recording, dtype=np.int16).reshape(-1, 1)

        wav_file_path = '/tmp/latest_audio.wav'
        write(wav_file_path, self.sample_rate, recording_array)

        with open(wav_file_path, "rb") as audio_file:
            transcript = self.openai.audio.transcriptions.create(
                model=self.model, 
                file=audio_file,
                response_format="text"
            )
        return transcript
    
def main(args=None):
    rclpy.init(args=args)
    speech2text_node = Speech2TextNode()
    rclpy.spin(speech2text_node)
    speech2text_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()