import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Pins connected to the HC-SR04 sensor
iTriggerPin = 23
iEchoPin    = 24

GPIO.setup(iTriggerPin, GPIO.OUT)
GPIO.setup(iEchoPin, GPIO.IN)

GPIO.output(iTriggerPin, False)
time.sleep(0.5)

while True:
      GPIO.output(iTriggerPin, True)
      time.sleep(0.0001)
      GPIO.output(iTriggerPin, False)

      while GPIO.input(iEchoPin) == 0:
            pass
      fPulseStart = time.time()

      while GPIO.input(iEchoPin) == 1:
            pass
      fPulseEnd = time.time()

      fPulseDuration = fPulseEnd - fPulseStart

      fDistance = round((fPulseDuration * 17150), 2)

      print "Distance:", fDistance, "cm."

      time.sleep(0.5)

GPIO.cleanup()