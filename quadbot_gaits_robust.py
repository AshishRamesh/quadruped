"""
Quadbot Gaits - Robust Version
Refactored to match the style of tests/servo_diagnostic.py using adafruit_servokit.
Includes proper I2C locking and reduced update frequency to prevent crashes.
"""
import time
import RPi.GPIO as GPIO
from threading import Thread, Lock
from adafruit_servokit import ServoKit

# --- Configuration & Constants ---

# GPIO Pin Definitions
leg1_s = 37
leg2_s = 35
leg3_s = 33
leg4_s = 31

# Movement Delays
# INCREASED delay to prevent I2C bus flooding/crashing
# 0.005s = 200Hz max update rate (much safer than 2000Hz)
move_delay = 0.005 
step_delay = 0.001

# Leg Offsets
leg1_offset = [0, 0, 0]
leg2_offset = [0, 10, 0]
leg3_offset = [0, 0, -10]
leg4_offset = [0, 0, -10]

# Leg Positions
front_lateral = 40
front_parallel = 90
front_lateral_add = -30

back_lateral = 140
back_parallel = 90
back_lateral_add = 30

footup = 0
footdown = 60

pincer_up = 130
pincer_down = 120

leg1_footdown = footdown
leg2_footdown = footdown
leg3_footdown = footdown
leg4_footdown = footdown

# Global State
leg_formation = 0
# We track current angles to do smooth incremental movement
channel_cur = [90] * 12

# Locks
# Critical for avoiding I2C collisions when multiple threads are running
i2c_mutex = Lock()

# --- Initialization ---

# 1. Setup GPIO
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass # Already set

GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 2. Setup ServoKit (Same as diagnostic)
try:
    kit = ServoKit(channels=16)
except Exception as e:
    print(f"Error initializing ServoKit: {e}")
    raise

# 3. Configure Servos (Same as diagnostic)
print("Configuring servos...")
for i in range(16):
    kit.servo[i].actuation_range = 180
print("Servos configured.")


# --- Helper Functions ---

def set_angle_safe(channel, angle):
    """
    Safely set servo angle using Mutex to prevent I2C collisions.
    Clamps angle to 0-180.
    """
    # Clamp
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = angle
        except Exception as e:
            print(f"Error setting servo {channel}: {e}")

def set_angle_safe_invert(channel, angle):
    """
    Safely set inverted servo angle.
    """
    # Clamp
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    # Invert: 0->180, 180->0
    inverted_angle = 180 - angle
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = inverted_angle
        except Exception as e:
            print(f"Error setting servo {channel} (invert): {e}")

# --- Leg Movement Functions ---
# Note: These use while loops to move "smoothly" by incrementing 1 degree at a time.
# We keep this logic but use the safe setters and slower delay.

def leg1(angle1, angle2, angle3):
    angle1 = int(angle1 + leg1_offset[0])
    angle2 = int(angle2 + leg1_offset[1])
    angle3 = int(angle3 + leg1_offset[2])

    while (channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3):
        # Joint 1
        if angle1 > channel_cur[0]:
            channel_cur[0] += 1
            set_angle_safe_invert(0, channel_cur[0])
        elif angle1 < channel_cur[0]:
            channel_cur[0] -= 1
            set_angle_safe_invert(0, channel_cur[0])

        # Joint 2
        if angle2 > channel_cur[1]:
            channel_cur[1] += 1
            set_angle_safe_invert(1, channel_cur[1])
        elif angle2 < channel_cur[1]:
            channel_cur[1] -= 1
            set_angle_safe_invert(1, channel_cur[1])

        # Joint 3
        if angle3 > channel_cur[2]:
            channel_cur[2] += 1
            set_angle_safe(2, channel_cur[2])
        elif angle3 < channel_cur[2]:
            channel_cur[2] -= 1
            set_angle_safe(2, channel_cur[2])

        time.sleep(move_delay)

def leg2(angle1, angle2, angle3):
    angle1 = int(angle1 + leg2_offset[0])
    angle2 = int(angle2 + leg2_offset[1])
    angle3 = int(angle3 + leg2_offset[2])

    while (channel_cur[3] != angle1 or channel_cur[4] != angle2 or channel_cur[5] != angle3):
        # Joint 1
        if angle1 > channel_cur[3]:
            channel_cur[3] += 1
            set_angle_safe_invert(3, channel_cur[3])
        elif angle1 < channel_cur[3]:
            channel_cur[3] -= 1
            set_angle_safe_invert(3, channel_cur[3])

        # Joint 2
        if angle2 > channel_cur[4]:
            channel_cur[4] += 1
            set_angle_safe_invert(4, channel_cur[4])
        elif angle2 < channel_cur[4]:
            channel_cur[4] -= 1
            set_angle_safe_invert(4, channel_cur[4])

        # Joint 3
        if angle3 > channel_cur[5]:
            channel_cur[5] += 1
            set_angle_safe(5, channel_cur[5])
        elif angle3 < channel_cur[5]:
            channel_cur[5] -= 1
            set_angle_safe(5, channel_cur[5])

        time.sleep(move_delay)

def leg3(angle1, angle2, angle3):
    angle1 = int(angle1 + leg3_offset[0])
    angle2 = int(angle2 + leg3_offset[1])
    angle3 = int(angle3 + leg3_offset[2])

    while (channel_cur[6] != angle1 or channel_cur[7] != angle2 or channel_cur[8] != angle3):
        # Joint 1
        if angle1 > channel_cur[6]:
            channel_cur[6] += 1
            set_angle_safe(6, channel_cur[6])
        elif angle1 < channel_cur[6]:
            channel_cur[6] -= 1
            set_angle_safe(6, channel_cur[6])

        # Joint 2
        if angle2 > channel_cur[7]:
            channel_cur[7] += 1
            set_angle_safe_invert(7, channel_cur[7])
        elif angle2 < channel_cur[7]:
            channel_cur[7] -= 1
            set_angle_safe_invert(7, channel_cur[7])

        # Joint 3
        if angle3 > channel_cur[8]:
            channel_cur[8] += 1
            set_angle_safe(8, channel_cur[8])
        elif angle3 < channel_cur[8]:
            channel_cur[8] -= 1
            set_angle_safe(8, channel_cur[8])

        time.sleep(move_delay)

def leg4(angle1, angle2, angle3):
    angle1 = int(angle1 + leg4_offset[0])
    angle2 = int(angle2 + leg4_offset[1])
    angle3 = int(angle3 + leg4_offset[2])

    while (channel_cur[9] != angle1 or channel_cur[10] != angle2 or channel_cur[11] != angle3):
        # Joint 1
        if angle1 > channel_cur[9]:
            channel_cur[9] += 1
            set_angle_safe(9, channel_cur[9])
        elif angle1 < channel_cur[9]:
            channel_cur[9] -= 1
            set_angle_safe(9, channel_cur[9])

        # Joint 2
        if angle2 > channel_cur[10]:
            channel_cur[10] += 1
            set_angle_safe_invert(10, channel_cur[10])
        elif angle2 < channel_cur[10]:
            channel_cur[10] -= 1
            set_angle_safe_invert(10, channel_cur[10])

        # Joint 3
        if angle3 > channel_cur[11]:
            channel_cur[11] += 1
            set_angle_safe(11, channel_cur[11])
        elif angle3 < channel_cur[11]:
            channel_cur[11] -= 1
            set_angle_safe(11, channel_cur[11])

        time.sleep(move_delay)

# --- Gait Functions ---

def begin():
    global leg_formation
    print("Beginning stance...")
    # Initialize all legs to start position
    leg1(89, 89, 89)
    leg2(89, 89, 89)
    leg3(89, 89, 89)
    leg4(89, 89, 89)

    time.sleep(1)

    leg1(front_parallel, footdown, pincer_down)
    leg2(back_parallel, footdown, pincer_down)
    leg3(back_lateral, footdown, pincer_down)
    leg4(front_lateral, footdown, pincer_down)

    leg_formation = 1
    print("Stance complete.")

def forward():
    global leg_formation
    if leg_formation == 1:
        # Lift leg1
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t2 = Thread(target=leg2, args=(back_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_parallel, footdown, pincer_down))

        t2.start()
        t3.start()
        t4.start()

        t2.join()
        t3.join()
        t4.join()

        # Lift leg3
        leg3(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)
    
    elif leg_formation == 2:
        # Lift leg4
        leg4(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t3 = Thread(target=leg3, args=(back_lateral, footdown, pincer_down))
        t2 = Thread(target=leg2, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        t1 = Thread(target=leg1, args=(front_parallel, footdown, pincer_down))
        
        t3.start()
        t2.start()
        t1.start()

        t3.join()
        t2.join()
        t1.join()
        time.sleep(step_delay)

        # Lift leg2
        leg2(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    # Toggle formation
    leg_formation = 2 if leg_formation == 1 else 1

def backward():
    global leg_formation
    if leg_formation == 1:
        # lift leg2
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        # send leg1 to lateral, and leg3 to parallel,
        t1 = Thread(target=leg1, args=(front_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral + front_lateral_add, footdown, pincer_down))

        t1.start()
        t3.start()
        t4.start()

        t1.join()
        t3.join()
        t4.join()

        # lift leg4
        leg4(front_lateral + front_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    elif leg_formation == 2:
        # lift leg3
        leg3(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t4 = Thread(target=leg4, args=(front_lateral, footdown, pincer_down))
        t2 = Thread(target=leg2, args=(back_parallel, footdown, pincer_down))
        t1 = Thread(target=leg1, args=(front_lateral + front_lateral_add, footdown, pincer_down))
        
        t4.start()
        t2.start()
        t1.start()

        t4.join()
        t2.join()
        t1.join()
        time.sleep(step_delay)

        # lift leg1
        leg1(front_lateral + front_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    leg_formation = 2 if leg_formation == 1 else 1

def turn_left():
    global leg_formation
    if leg_formation == 1:
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral, footup, pincer_up)
        time.sleep(step_delay)

        t1 = Thread(target=leg1, args=(front_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_parallel, footdown, pincer_down))

        t1.start()
        t3.start()
        t4.start()

        leg2(back_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t1.join()
        t3.join()
        t4.join()

        leg3(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    elif leg_formation == 2:
        leg4(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral, footup, pincer_up)
        time.sleep(step_delay)

        t3 = Thread(target=leg3, args=(back_lateral, footdown, pincer_down))
        t2 = Thread(target=leg2, args=(back_parallel, footdown, pincer_down))
        t1 = Thread(target=leg1, args=(front_lateral + front_lateral_add, footdown, pincer_down))
        
        t3.start()
        t2.start()
        t1.start()

        leg4(front_lateral, footdown, pincer_down)

        t3.join()
        t2.join()
        t1.join()
        time.sleep(step_delay)

        leg1(front_lateral + front_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    leg_formation = 2 if leg_formation == 1 else 1

def turn_right():
    global leg_formation
    if leg_formation == 1:
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footup, pincer_up)
        time.sleep(step_delay)

        t2 = Thread(target=leg2, args=(back_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral + front_lateral_add, footdown, pincer_down))

        t2.start()
        t3.start()
        t4.start()

        leg1(front_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t2.join()
        t3.join()
        t4.join()

        leg4(front_lateral + front_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    elif leg_formation == 2:
        leg3(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral, footup, pincer_up)
        time.sleep(step_delay)

        t1 = Thread(target=leg1, args=(front_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral, footdown, pincer_down))
        t2 = Thread(target=leg2, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        
        t1.start()
        t4.start()
        t2.start()

        leg3(back_lateral, footdown, pincer_down)

        t1.join()
        t4.join()
        t2.join()
        time.sleep(step_delay)

        leg2(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    leg_formation = 2 if leg_formation == 1 else 1

# --- Main ---

def main():
    print("Starting Quadbot Gaits (Robust)")
    try:
        begin()
        
        print("Sequence: Turn Right x4")
        turn_right()
        turn_right()
        turn_right()
        turn_right()

        # Uncomment to test forward
        # for _ in range(5):
        #     forward()

    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        print("Cleaning up...")
        # Reset to safe position before exit?
        # Maybe just relax?
        GPIO.cleanup()

if __name__ == '__main__':
    main()
