import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray

import sounddevice as sd
import RPi.GPIO as GPIO

import time
import pyaudio
import wave
import sys
import random
from scipy.io import wavfile


# Main gpio pin assignments
LED = 13
BELL_PWM = 5
BELL_DIR = 3
PHONE_SWITCH = 7
COIN_SLOT = 36


# Executes a single "riiiiing"
def ring():
    ringFreq = 1/20 #Timing to adjust ring frequency
    ringTime = 8		#Length of a single ring (in iterations)
    GPIO.output(BELL_PWM,GPIO.HIGH) #Set pwm pin high to deliver current
    for i in range(ringTime):
        # Alternate solenoid direction 
        GPIO.output(BELL_DIR,GPIO.LOW)
        time.sleep(ringFreq)
        GPIO.output(BELL_DIR,GPIO.HIGH)
        time.sleep(ringFreq)
    # After finishing the ring, turn off current to solenoid
    GPIO.output(BELL_PWM,GPIO.LOW)


# Function to flash LED for a random time period
def flashLED():
    dt = 0.5
    numFlashes = random.randint(2,4)
    for i in range(numFlashes):
        GPIO.output(LED,GPIO.HIGH)
        time.sleep(dt)
        GPIO.output(LED,GPIO.LOW)
        time.sleep(dt)

class PhoneOperatorNode(Node):
                
        def __init__(self):
            super().__init__('phone_operator_node')
    
            # TODO: Initialize GPIO pins
            # TODO: Initialize coin trigger
            # TODO: Initialize phone pickup trigger
                # Initialize pinmodes
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(LED,GPIO.OUT)
            GPIO.setup(BELL_PWM,GPIO.OUT)
            GPIO.setup(BELL_DIR,GPIO.OUT)
            GPIO.setup(PHONE_SWITCH,GPIO.IN,pull_up_down=GPIO.PUD_UP)
            GPIO.setup(COIN_SLOT,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

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
            self.run()
            
        def run(self):
            self.coin_trigger()

        def coin_trigger(self):
            # Wait until a coin is detected:
            while(GPIO.input(COIN_SLOT)):
                time.sleep(0.001)
            self.get_logger().info(f'Coin Inserted')
            self.ring_phone()

        def ring_phone(self):
            flashLED()
            # Ring phone until phone is picked up or it times out
            # First check to see if phone is on the hook:
            hook = GPIO.input(PHONE_SWITCH)
            startTime = time.time()
            timeRinging = 0
            while(not hook and timeRinging < 8):
                GPIO.output(LED,GPIO.HIGH)  #LED On
                ring()  #Ring on
                GPIO.output(LED,GPIO.LOW)#LED Off
                time.sleep(1)   #Pause until next ring
                timeRinging = time.time() - startTime   #Calculate time ringing
                hook = GPIO.input(PHONE_SWITCH) #Check if phone is picked up
                self.get_logger().info(f'Phone Ringing')
            self.play_operator()

            
        def play_operator(self):
            time.sleep(1)
            voiceRecording = "/app/src/recordings/"+str(random.randint(1,3))+".wav"
            sample_rate, data = wavfile.read(voiceRecording)
            sd.play(data,self.sample_rate,device=self.device_id)
            sd.wait()

            # Play the beep
            beep = "/app/src/recordings/Beep.wav"
            sample_rate, data = wavfile.read(beep)
            sd.play(data,self.sample_rate,device=self.device_id)
            sd.wait()



            # sd.play(operator_message, device=self.device_id, samplerate=self.sample_rate, channels=1)
            self.record_audio()

        def record_audio(self):
            self.get_logger().info(f'Recording Audio')
            GPIO.output(LED,GPIO.HIGH)
            msg = Empty()
            self.publisher.publish(msg)

        def audio_callback(self, msg : Int16MultiArray):
            self.get_logger().info(f'Audio Callback')
            self.play_end_call()

        def play_end_call(self):
            GPIO.output(LED,GPIO.LOW)
            self.get_logger().info(f'End Call Playing')
            beep = "/app/src/recordings/Beep.wav"
            sample_rate, data = wavfile.read(beep)
            sd.play(data,self.sample_rate,device=self.device_id)
            sd.wait()
            self.run()



def main(args=None):
    rclpy.init(args=args)
    node = PhoneOperatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()