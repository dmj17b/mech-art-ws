import RPi.GPIO as GPIO
import time
import random

LED = 13
BELL_PWM = 5
BELL_DIR = 3
PHONE_SWITCH = 7

# Function to flash LED for a random time period
def flashLED():
    dt = 0.5
    numFlashes = random.randint(3,8)
    for i in range(numFlashes):
        GPIO.output(LED,GPIO.HIGH)
        print("ON")
        time.sleep(dt)
        GPIO.output(LED,GPIO.LOW)
        print("Off")
        time.sleep(dt)
        
def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED,GPIO.OUT)
    GPIO.setup(BELL_PWM,GPIO.OUT)
    GPIO.setup(BELL_DIR,GPIO.OUT)
    GPIO.setup(PHONE_SWITCH,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    
    flashLED()
    
if __name__=="__main__":
    main()