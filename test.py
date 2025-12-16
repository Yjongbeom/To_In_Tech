from RPi import GPIO
import time

PIN = 16  # BCM 번호

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(PIN, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(PIN, GPIO.LOW)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
