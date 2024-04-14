import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray

import sounddevice as sd
# import RPi.GPIO as GPIO

class PhoneOperatorNode(Node):
                
        def __init__(self):
            super().__init__('phone_operator_node')
    
            # TODO: Initialize GPIO pins
            # TODO: Initialize coin trigger
            # TODO: Initialize phone pickup trigger

            self.publisher = self.create_publisher(Empty, 'record_audio', 10)
            self.subscription = self.create_subscription(Int16MultiArray, 'audio_data', self.audio_callback, 10)
    
            self.declare_parameter('device_id', -1)
            self.device_id = self.get_parameter('device_id').value

            self.declare_parameter('sample_rate', 44100)
            self.sample_rate = self.get_parameter('sample_rate').value


            device_info = sd.query_devices(self.device_id, 'input')
            self.get_logger().info(f'\n'
                f'\t---Phone Operator Node---\n'
                f'\t Device ID: {self.device_id}\n'
                f'\t Device Name: {device_info["name"]}\n'
                f'\t Channels: {device_info["max_input_channels"]}\n'
                f'\t Default Sample Rate: {device_info["default_samplerate"]}\n'
                f'\t--------------------------\n'
            )

        def coin_trigger(self):
            self.get_logger().info(f'Coin Inserted')
            self.ring_phone()

        def ring_phone(self):
            self.get_logger().info(f'Phone Ringing')
            # TODO: Ring the phone
            
        def phone_pickup_trigger(self):
            self.get_logger().info(f'Phone Picked Up')
            # TODO: Stop ringing the phone
            self.play_operator()
            
        def play_operator(self):
            self.get_logger().info(f'Operator Playing')
            # TODO: Play operator message
            # sd.play(operator_message, device=self.device_id, samplerate=self.sample_rate, channels=1)
            self.record_audio()

        def record_audio(self):
            self.get_logger().info(f'Recording Audio')
            msg = Empty()
            self.publisher.publish(msg)

        def audio_callback(self, msg : Int16MultiArray):
            self.get_logger().info(f'Audio Callback')
            self.play_end_call()

        def play_end_call(self):
            self.get_logger().info(f'End Call Playing')
            # TODO: Play end call message

def main(args=None):
    rclpy.init(args=args)
    node = PhoneOperatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()