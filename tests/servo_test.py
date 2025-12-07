#!/usr/bin/env python3
"""
Servo Sweep Test
Tests 4 servos by sweeping them from 0 to 180 degrees and back
"""

from adafruit_servokit import ServoKit
import time

# Initialize ServoKit
kit = ServoKit(channels=16)

# Configuration
NUM_SERVOS = 4
MIN_ANGLE = 0
MAX_ANGLE = 180
STEP = 5  # degrees per step
DELAY = 0.05  # seconds between steps

print("=" * 50)
print("Servo Sweep Test - 4 Servos")
print("=" * 50)
print(f"Sweeping servos 0-{NUM_SERVOS-1}")
print(f"Range: {MIN_ANGLE}° to {MAX_ANGLE}°")
print(f"Step: {STEP}°")
print()

try:
    # Sweep forward (0 to 180)
    print("Sweeping forward (0° → 180°)...")
    for angle in range(MIN_ANGLE, MAX_ANGLE + 1, STEP):
        for servo_num in range(NUM_SERVOS):
            kit.servo[servo_num].angle = angle
        print(f"  Angle: {angle}°", end='\r')
        time.sleep(DELAY)
    
    print(f"\n  All servos at {MAX_ANGLE}°")
    time.sleep(0.5)
    
    # Sweep backward (180 to 0)
    print("Sweeping backward (180° → 0°)...")
    for angle in range(MAX_ANGLE, MIN_ANGLE - 1, -STEP):
        for servo_num in range(NUM_SERVOS):
            kit.servo[servo_num].angle = angle
        print(f"  Angle: {angle}°", end='\r')
        time.sleep(DELAY)
    
    print(f"\n  All servos at {MIN_ANGLE}°")
    
    print()
    print("=" * 50)
    print("Sweep test complete!")
    print("=" * 50)

except KeyboardInterrupt:
    print("\n\nTest interrupted by user")
    # Return all servos to neutral position
    print("Returning servos to 90°...")
    for servo_num in range(NUM_SERVOS):
        kit.servo[servo_num].angle = 90
    print("Done")

except Exception as e:
    print(f"\nError: {e}")
    # Return all servos to neutral position
    print("Returning servos to 90°...")
    for servo_num in range(NUM_SERVOS):
        kit.servo[servo_num].angle = 90
    print("Done")