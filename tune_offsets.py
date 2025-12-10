import board
import busio
from adafruit_pca9685 import PCA9685
import time

# --- Configuration ---

# Initial Offsets (from your current code)
# Format: [Leg1, Leg2, Leg3, Leg4]
# Each Leg: [Angle1, Angle2, Angle3]
offsets = [
    [0, 0, 0],    # Leg 1
    [0, 10, 0],   # Leg 2
    [0, 0, -10],  # Leg 3
    [0, 0, -10]   # Leg 4
]

# Servo Inversion Map
# True = setServo_invert, False = setServo
# Based on leg1(), leg2(), leg3(), leg4() functions
inversion_map = [
    [True, True, False],   # Leg 1: Ch 0, 1, 2
    [True, True, False],   # Leg 2: Ch 3, 4, 5
    [False, True, False],  # Leg 3: Ch 6, 7, 8
    [False, True, False]   # Leg 4: Ch 9, 10, 11
]

# Channel Map
channel_map = [
    [0, 1, 2],
    [3, 4, 5],
    [6, 7, 8],
    [9, 10, 11]
]

# Default "Home" Angles to tune around
home_angles = [90, 90, 90]

# --- Hardware Init ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# --- Helper Functions ---

def set_servo_pulse(channel, pulse_old):
    """Sets servo pulse using old 0-4096 scale converted to 16-bit duty cycle."""
    pulse_old = max(0, min(4096, pulse_old))
    duty_cycle = int(pulse_old * 65535 / 4096)
    try:
        pca.channels[channel].duty_cycle = duty_cycle
    except OSError:
        pass

def write_servo(channel, angle, invert):
    """Writes angle to servo, handling inversion logic."""
    if angle < 0: angle = 0
    if angle > 180: angle = 180

    if invert:
        # setServo_invert logic: (angle * -2.5) + 600
        pulse = int((angle * -2.5) + 600)
    else:
        # setServo logic: (angle * 2.5) + 150
        pulse = int((angle * 2.5) + 150)
    
    set_servo_pulse(channel, pulse)

def update_leg(leg_idx):
    """Updates all servos for a specific leg based on current offsets."""
    for joint_idx in range(3):
        channel = channel_map[leg_idx][joint_idx]
        invert = inversion_map[leg_idx][joint_idx]
        offset = offsets[leg_idx][joint_idx]
        base_angle = home_angles[joint_idx]
        
        # Apply offset
        final_angle = base_angle + offset
        write_servo(channel, final_angle, invert)

def print_offsets():
    print("\nCurrent Offsets:")
    print(f"leg1_offset = {offsets[0]}")
    print(f"leg2_offset = {offsets[1]}")
    print(f"leg3_offset = {offsets[2]}")
    print(f"leg4_offset = {offsets[3]}")
    print("-" * 20)

def main():
    print("=== Quadbot Offset Tuner ===")
    print("This tool will help you tune the offsets for each leg.")
    print("The robot will move to a 'Home' position (90, 90, 90) + Offsets.")
    
    # Move all to home initially
    for i in range(4):
        update_leg(i)

    while True:
        print_offsets()
        try:
            leg_in = input("Select Leg (1-4) or 'q' to quit: ").strip().lower()
            if leg_in == 'q':
                break
            
            leg_idx = int(leg_in) - 1
            if not (0 <= leg_idx <= 3):
                print("Invalid leg number.")
                continue

            while True:
                print(f"\nEditing Leg {leg_idx + 1}")
                print(f"Current Offsets: {offsets[leg_idx]}")
                joint_in = input("Select Joint (1-3) or 'b' to back: ").strip().lower()
                
                if joint_in == 'b':
                    break
                
                joint_idx = int(joint_in) - 1
                if not (0 <= joint_idx <= 2):
                    print("Invalid joint number.")
                    continue

                print(f"\nAdjusting Leg {leg_idx + 1}, Joint {joint_idx + 1}")
                print("Controls: '+' to increase, '-' to decrease, 'Enter' to finish joint")
                
                while True:
                    curr_offset = offsets[leg_idx][joint_idx]
                    print(f"Current Offset: {curr_offset}")
                    cmd = input("Command (+/-): ").strip()
                    
                    if cmd == '+':
                        offsets[leg_idx][joint_idx] += 1
                    elif cmd == '-':
                        offsets[leg_idx][joint_idx] -= 1
                    elif cmd == '':
                        break
                    else:
                        print("Invalid command.")
                    
                    # Update servo immediately
                    update_leg(leg_idx)

        except ValueError:
            print("Invalid input, please enter a number.")
        except KeyboardInterrupt:
            break

    print("\nFinal Configuration:")
    print_offsets()
    print("Copy these lines into your 'quadbot_gaits_robust.py' file.")
    pca.deinit()

if __name__ == '__main__':
    main()
