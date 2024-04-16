import RPi.GPIO as GPIO
import time

LED = 15
BELL_PWM = 5
BELL_DIR = 3
PHONE_SWITCH = 18

# Executes a single "riiiiing"
def ring():
    ringFreq = 1/20 #Timing to adjust ring frequency
    ringTime = 8	#Length of a single ring (in iterations)
    GPIO.output(BELL_PWM,GPIO.HIGH) #Set pwm pin high to deliver current
    for i in range(ringTime):
        # Alternate solenoid direction 
        GPIO.output(BELL_DIR,GPIO.LOW)
        time.sleep(ringFreq)
        GPIO.output(BELL_DIR,GPIO.HIGH)
        time.sleep(ringFreq)
    # After finishing the ring, turn off current to solenoid
    GPIO.output(BELL_PWM,GPIO.LOW)
    
def main():
    # Initialize pinmodes
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED,GPIO.OUT)
    GPIO.setup(BELL_PWM,GPIO.OUT)
    GPIO.setup(BELL_DIR,GPIO.OUT)
    GPIO.setup(PHONE_SWITCH,GPIO.IN)
    
    ring()
    time.sleep(1)
    ring()
    
if __name__ == "__main__":
    main()