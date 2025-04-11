from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP1, eQEP2
import time

encoder = RotaryEncoder(eQEP2)
print(encoder.enabled)
counter = 2
last_counter = 0
while True:
    counter = encoder.position
    print(counter)
    angle = counter * 360/360
    if counter != last_counter:
        print("Angle: {:.2f}".format(angle))
        last_counter = counter
    time.sleep(1)
