"""
Quadbot Gaits - Debug Mode
Includes extensive print statements to trace execution and identify crash points.
"""
import time
import sys
import RPi.GPIO as GPIO
from threading import Thread, Lock
from adafruit_servokit import ServoKit

# --- Configuration & Constants ---

# GPIO Pin Definitions
leg1_s = 37
leg2_s = 35
leg3_s = 33
leg4_s = 31

move_delay = 0.02
step_delay = 0.2
STEP_SIZE = 4

leg1_offset = [0, 0, 0]
leg2_offset = [0, 10, 0]
leg3_offset = [0, 0, -10]
leg4_offset = [0, 0, -10]

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

leg_formation = 0
channel_cur = [90] * 12

i2c_mutex = Lock()

# --- Logging Helper ---
def log(msg):
    """Print with timestamp to help trace timing."""
    # Force flush to ensure output appears before a hard crash
    print(f"[{time.time():.3f}] {msg}", flush=True)

# --- Initialization ---

log("Initializing GPIO...")
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass 

GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
log("GPIO Initialized.")

log("Initializing ServoKit...")
try:
    kit = ServoKit(channels=16)
except Exception as e:
    log(f"CRITICAL ERROR initializing ServoKit: {e}")
    raise

log("Configuring Servos...")
for i in range(16):
    kit.servo[i].actuation_range = 180
log("Servos configured.")


# --- Helper Functions ---

def set_angle_safe(channel, angle):
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    with i2c_mutex:
        try:
            # log(f"  -> Servo {channel} set to {angle}") # Very verbose, uncomment if needed
            kit.servo[channel].angle = angle
            time.sleep(0.001) 
        except Exception as e:
            log(f"ERROR setting servo {channel}: {e}")

def set_angle_safe_invert(channel, angle):
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    
    inverted_angle = 180 - angle
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = inverted_angle
            time.sleep(0.001)
        except Exception as e:
            log(f"ERROR setting servo {channel} (invert): {e}")

def move_towards(current, target, step):
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

# --- Leg Movement Functions ---

def leg1(angle1, angle2, angle3):
    log(f"Leg 1 moving to ({angle1}, {angle2}, {angle3})")
    angle1 = int(angle1 + leg1_offset[0])
    angle2 = int(angle2 + leg1_offset[1])
    angle3 = int(angle3 + leg1_offset[2])

    while (channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3):
        old_vals = list(channel_cur)
        channel_cur[0] = move_towards(channel_cur[0], angle1, STEP_SIZE)
        channel_cur[1] = move_towards(channel_cur[1], angle2, STEP_SIZE)
        channel_cur[2] = move_towards(channel_cur[2], angle3, STEP_SIZE)
        
        if channel_cur[0] != old_vals[0]: set_angle_safe_invert(0, channel_cur[0])
        if channel_cur[1] != old_vals[1]: set_angle_safe_invert(1, channel_cur[1])
        if channel_cur[2] != old_vals[2]: set_angle_safe(2, channel_cur[2])
        time.sleep(move_delay)
    log("Leg 1 reached target")

def leg2(angle1, angle2, angle3):
    log(f"Leg 2 moving to ({angle1}, {angle2}, {angle3})")
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
    log("Leg 2 reached target")

def leg3(angle1, angle2, angle3):
    log(f"Leg 3 moving to ({angle1}, {angle2}, {angle3})")
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
    log("Leg 3 reached target")

def leg4(angle1, angle2, angle3):
    log(f"Leg 4 moving to ({angle1}, {angle2}, {angle3})")
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
    log("Leg 4 reached target")

# --- Gait Functions ---

def begin():
    global leg_formation
    log("=== BEGIN STANCE START ===")
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
    log("=== BEGIN STANCE COMPLETE ===")

def turn_right():
    global leg_formation
    log(f"=== TURN RIGHT START (Formation: {leg_formation}) ===")
    
    if leg_formation == 1:
        log("Phase 1: Moves for Formation 1")
        leg1(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral, footup, pincer_up)
        time.sleep(step_delay)

        log("Starting thread group 1 (L2, L3, L4)")
        t2 = Thread(target=leg2, args=(back_lateral, footdown, pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral + front_lateral_add, footdown, pincer_down))

        t2.start()
        t3.start()
        t4.start()

        leg1(front_lateral, footdown, pincer_down)
        time.sleep(step_delay)

        log("Waiting for thread group 1 to join...")
        t2.join()
        t3.join()
        t4.join()
        log("Thread group 1 joined.")

        leg4(front_lateral + front_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    elif leg_formation == 2:
        log("Phase 2: Moves for Formation 2")
        leg3(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral, footup, pincer_up)
        time.sleep(step_delay)

        log("Starting thread group 2 (L1, L4, L2)")
        t1 = Thread(target=leg1, args=(front_parallel, footdown, pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral, footdown, pincer_down))
        t2 = Thread(target=leg2, args=(back_lateral + back_lateral_add, footdown, pincer_down))
        
        t1.start()
        t4.start()
        t2.start()

        leg3(back_lateral, footdown, pincer_down)

        log("Waiting for thread group 2 to join...")
        t1.join()
        t4.join()
        t2.join()
        time.sleep(step_delay)
        log("Thread group 2 joined.")

        leg2(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)

    leg_formation = 2 if leg_formation == 1 else 1
    log("=== TURN RIGHT COMPLETE ===")

# --- Main ---

def main():
    log("Starting Quadbot Gaits (Debug/Verbose)")
    try:
        begin()
        
        log("Sequence: Turn Right x4 loop starting")
        for i in range(4):
            log(f">> Loop iteration {i+1}")
            turn_right()
            time.sleep(0.5) # Cool down between major steps
        
        log("Sequence Complete.")

    except KeyboardInterrupt:
        log("Interrupted by user.")
    except Exception as e:
        log(f"CRITICAL EXCEPTION in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        log("Cleaning up...")
        GPIO.cleanup()
        log("Cleanup finished. Exiting.")

if __name__ == '__main__':
    main()
