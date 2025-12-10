"""
Quadbot Gaits - Robust & Optimized Version
Refactored to reduce I2C traffic and power consumption.
Changes:
- Increased delays between movements.
- Increased servo step size (reduces total I2C writes).
- Added I2C locking.
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
# Significantly increased to reduce I2C bus load and power spikes
move_delay = 0.02  # 20ms per update (50 updates/sec)
step_delay = 0.2   # 200ms pause between major moves

# Servo Movement Optimization
# Moving 4 degrees at a time instead of 1 reduces I2C traffic by 75%
STEP_SIZE = 4

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
# Initiate all at 90
channel_cur = [90] * 12

# Locks
i2c_mutex = Lock()

# --- Initialization ---

# 1. Setup GPIO
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass 

GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 2. Setup ServoKit 
try:
    kit = ServoKit(channels=16)
except Exception as e:
    print(f"Error initializing ServoKit: {e}")
    raise

# 3. Configure Servos
print("Configuring servos...")
for i in range(16):
    kit.servo[i].actuation_range = 180
print("Servos configured.")


# --- Helper Functions ---

def set_angle_safe(channel, angle):
    """Safely set servo angle using Mutex and clamping."""
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = angle
            # Tiny sleep to ensure bus is clear
            time.sleep(0.001) 
        except Exception as e:
            print(f"Error setting servo {channel}: {e}")

def set_angle_safe_invert(channel, angle):
    """Safely set inverted servo angle."""
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    inverted_angle = 180 - angle
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = inverted_angle
            time.sleep(0.001)
        except Exception as e:
            print(f"Error setting servo {channel} (invert): {e}")

def move_towards(current, target, step):
    """Helper to calculate next step towards target."""
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

# --- Leg Movement Functions ---

def leg1(angle1, angle2, angle3):
    angle1 = int(angle1 + leg1_offset[0])
    angle2 = int(angle2 + leg1_offset[1])
    angle3 = int(angle3 + leg1_offset[2])

    while (channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3):
        # Update current values towards target
        old_vals = list(channel_cur) # Snapshop for check
        
        # Calculate new positions
        channel_cur[0] = move_towards(channel_cur[0], angle1, STEP_SIZE)
        channel_cur[1] = move_towards(channel_cur[1], angle2, STEP_SIZE)
        channel_cur[2] = move_towards(channel_cur[2], angle3, STEP_SIZE)
        
        # Only write to I2C if value changed
        if channel_cur[0] != old_vals[0]:
            set_angle_safe_invert(0, channel_cur[0])
        if channel_cur[1] != old_vals[1]:
            set_angle_safe_invert(1, channel_cur[1])
        if channel_cur[2] != old_vals[2]:
            set_angle_safe(2, channel_cur[2])

        time.sleep(move_delay)

def leg2(angle1, angle2, angle3):
    angle1 = int(angle1 + leg2_offset[0])
    angle2 = int(angle2 + leg2_offset[1])
    angle3 = int(angle3 + leg2_offset[2])

    while (channel_cur[3] != angle1 or channel_cur[4] != angle2 or channel_cur[5] != angle3):
        old_vals = list(channel_cur)

        channel_cur[3] = move_towards(channel_cur[3], angle1, STEP_SIZE)
        channel_cur[4] = move_towards(channel_cur[4], angle2, STEP_SIZE)
        channel_cur[5] = move_towards(channel_cur[5], angle3, STEP_SIZE)

        if channel_cur[3] != old_vals[3]: set_angle_safe_invert(3, channel_cur[3])
        if channel_cur[4] != old_vals[4]: set_angle_safe_invert(4, channel_cur[4])
        if channel_cur[5] != old_vals[5]: set_angle_safe(5, channel_cur[5])

        time.sleep(move_delay)

def leg3(angle1, angle2, angle3):
    angle1 = int(angle1 + leg3_offset[0])
    angle2 = int(angle2 + leg3_offset[1])
    angle3 = int(angle3 + leg3_offset[2])

    while (channel_cur[6] != angle1 or channel_cur[7] != angle2 or channel_cur[8] != angle3):
        old_vals = list(channel_cur)
        
        channel_cur[6] = move_towards(channel_cur[6], angle1, STEP_SIZE)
        channel_cur[7] = move_towards(channel_cur[7], angle2, STEP_SIZE)
        channel_cur[8] = move_towards(channel_cur[8], angle3, STEP_SIZE)

        if channel_cur[6] != old_vals[6]: set_angle_safe(6, channel_cur[6])
        if channel_cur[7] != old_vals[7]: set_angle_safe_invert(7, channel_cur[7])
        if channel_cur[8] != old_vals[8]: set_angle_safe(8, channel_cur[8])

        time.sleep(move_delay)

def leg4(angle1, angle2, angle3):
    angle1 = int(angle1 + leg4_offset[0])
    angle2 = int(angle2 + leg4_offset[1])
    angle3 = int(angle3 + leg4_offset[2])

    while (channel_cur[9] != angle1 or channel_cur[10] != angle2 or channel_cur[11] != angle3):
        old_vals = list(channel_cur)
        
        channel_cur[9] = move_towards(channel_cur[9], angle1, STEP_SIZE)
        channel_cur[10] = move_towards(channel_cur[10], angle2, STEP_SIZE)
        channel_cur[11] = move_towards(channel_cur[11], angle3, STEP_SIZE)

        if channel_cur[9] != old_vals[9]: set_angle_safe(9, channel_cur[9])
        if channel_cur[10] != old_vals[10]: set_angle_safe_invert(10, channel_cur[10])
        if channel_cur[11] != old_vals[11]: set_angle_safe(11, channel_cur[11])

        time.sleep(move_delay)

# --- Gait Functions ---

def begin():
    global leg_formation
    print("Beginning stance...")
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
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t2 = Thread(target=leg2, args=(back_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_parallel, footdown, pincer_down))

        # Stagger start to separate power spikes
        t2.start()
        t3.start()
        t4.start()

        t2.join()
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

        leg2(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    leg_formation = 2 if leg_formation == 1 else 1

def backward():
    global leg_formation
    if leg_formation == 1:
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        t1 = Thread(target=leg1, args=(front_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral + front_lateral_add, footdown, pincer_down))

        t1.start()
        t3.start()
        t4.start()

        t1.join()
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
    print("Starting Quadbot Gaits (Optimized)")
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
        GPIO.cleanup()

if __name__ == '__main__':
    main()
