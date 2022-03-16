import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD) #GPIO.BCM
servo_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)
p = GPIO.PWM(servo_pin, 50)
p.start(7.5)


def moveservo():
    while True:
        angle = int(input("DegreeAngle:"))
        if angle<0 and angle>180:
            print("Try again")
        else:
            p.ChangeDutyCycle(2.5+5*(angle/90))

try:
    moveservo()

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()


    
