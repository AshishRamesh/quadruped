from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)
SERVO_INDEX = 0  # change if you want

def move(angle, delay=0.5):
    kit.servo[SERVO_INDEX].angle = angle
    time.sleep(delay)

print("Controls:")
print("Press ENTER  →  0 → 90")
print("Press \\      →  90 → 0 → 90 → 180 → 90")
print("Press q      →  quit")

while True:
    key = input(">>> ").strip()

    # ENTER key → input("") produces empty string
    if key == "":
        print("Running sequence A: 0 → 90")
        move(180)
        move(90)

    # Backslash key \ (since shift-right can't be read over terminal)
    elif key == "\\":
        print("Running sequence B: 90 → 0 → 90 → 180 → 90")
        move(90)
        move(180)
        move(90)
        move(120)
        move(90)

    elif key == "q":
        print("Exiting...")
        break

    else:
        print("Invalid key. Use ENTER or \\")
