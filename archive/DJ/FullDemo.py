import RPi.GPIO as GPIO
import time
import pyaudio
import wave
import sys
import random

# Main gpio pin assignments
LED = 13
BELL_PWM = 5
BELL_DIR = 3
PHONE_SWITCH = 7
COIN_SLOT = 36

# Setting up stuff for the audio playback
class AudioFile:
    chunk = 1024

    def __init__(self, file):
        """ Init audio stream """ 
        self.wf = wave.open(file, 'rb')
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format = self.p.get_format_from_width(self.wf.getsampwidth()),
            channels = self.wf.getnchannels(),
            rate = self.wf.getframerate(),
            output = True
        )

    def play(self):
        """ Play entire file """
        data = self.wf.readframes(self.chunk)
        while data != b'':
            self.stream.write(data)
            data = self.wf.readframes(self.chunk)

    def close(self):
        """ Graceful shutdown """ 
        self.stream.close()
        self.p.terminate()

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

################### MAIN #####################
def main():
    # Initialize pinmodes
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED,GPIO.OUT)
    GPIO.setup(BELL_PWM,GPIO.OUT)
    GPIO.setup(BELL_DIR,GPIO.OUT)
    GPIO.setup(PHONE_SWITCH,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(COIN_SLOT,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)


    while True:
        # Wait until a coin is detected:
        coinDetected = False
        while(coinDetected==False):
            coinDetected = not GPIO.input(COIN_SLOT)
            time.sleep(0.001)

        flashLED()    
            
        # First check to see if phone is on the hook:
        hook = GPIO.input(PHONE_SWITCH)

        # Ring phone until phone is picked up or it times out
        startTime = time.time()
        timeRinging = 0
        while(not hook and timeRinging < 15):
            GPIO.output(LED,GPIO.HIGH)  #LED On
            ring()  #Ring on
            GPIO.output(LED,GPIO.LOW)#LED Off
            time.sleep(1)   #Pause until next ring
            timeRinging = time.time() - startTime   #Calculate time ringing
            hook = GPIO.input(PHONE_SWITCH) #Check if phone is picked up

        time.sleep(1)
        # Play a random AI voice message
        voiceRecording = "recordings/"+str(random.randint(1,6))+".wav"
        a = AudioFile(voiceRecording)
        a.play()
        a.close()
        # Play the beep
        a = AudioFile("recordings/Beep.wav")
        a.play()
        a.close()

        print("Recording wish now...")
        GPIO.output(LED,GPIO.HIGH)
        time.sleep(5)
        print("Finished recording wish")
        GPIO.output(LED,GPIO.LOW)
    
    
if __name__ == "__main__":
    main()