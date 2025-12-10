#!/usr/bin/env python3
"""
Robust teleoperation script for quadruped robot with timeout protection
Use arrow keys to control robot movement
"""

from __future__ import division
import time
import sys
import tty
import termios
import signal
import RPi.GPIO as GPIO
from threading import Thread, Lock, Event
from adafruit_servokit import ServoKit

# Global shutdown event for clean exit
shutdown_event = Event()

cur_angle_mutex = Lock()
i2c_mutex = Lock()

# GPIO pin definitions for leg sensors
leg1_s = 37
leg2_s = 35
leg3_s = 33
leg4_s = 31

# Setup GPIO before ServoKit initialization to avoid mode conflicts
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass  # Mode already set, continue

GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize ServoKit for 16 channels
print("Initializing ServoKit...", flush=True)
kit = ServoKit(channels=16)

# Set actuation range
for i in range(16):
    kit.servo[i].actuation_range = 180

# Updated delays from quadbot_gaits_robust.py
move_delay = 0.0005
step_delay = 0.001

leg1_offset = [0,0,0]
leg2_offset = [0,10,0]
leg3_offset = [0,0,-10]
leg4_offset = [0,0,-10]

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

leg_formation = 0
servos_configured = False

channel_cur = [90]*12


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Interrupt received! Shutting down...", flush=True)
    shutdown_event.set()
    try:
        for i in range(12):
            setServo_safe(i, 90)
    except:
        pass
    GPIO.cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def clamp_angle(angle):
    """Ensure angle is within valid servo range"""
    return max(0, min(180, int(angle)))


def setServo_safe(channel, angle):
    """Set servo with error handling and bounds checking"""
    if shutdown_event.is_set():
        return False
    
    angle = clamp_angle(angle)
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = angle
            return True
        except Exception as e:
            # print(f"Error setting servo {channel}: {e}", flush=True)
            return False


def setServo_invert_safe(channel, angle):
    """Set inverted servo with error handling and bounds checking"""
    if shutdown_event.is_set():
        return False
    
    angle = clamp_angle(angle)
    inverted_angle = 180 - angle
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = inverted_angle
            return True
        except Exception as e:
            # print(f"Error setting servo {channel}: {e}", flush=True)
            return False


def getch():
    """Get a single character from stdin without echo"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_controls():
    """Print control instructions"""
    print("\n" + "="*50, flush=True)
    print("🤖 QUADRUPED TELEOPERATION - ROBUST VERSION", flush=True)
    print("="*50, flush=True)
    print("  ↑ (UP)    : Move Forward", flush=True)
    print("  ↓ (DOWN)  : Move Backward", flush=True)
    print("  ← (LEFT)  : Turn Left", flush=True)
    print("  → (RIGHT) : Turn Right", flush=True)
    print("  SPACE     : Stop/Rest position", flush=True)
    print("  ESC or q  : Quit", flush=True)
    print("="*50, flush=True)
    print("\nInitializing robot...\n", flush=True)


# Updated Leg Functions from quadbot_gaits_robust.py
# Added shutdown_event check inside the loop

def leg1(angle1, angle2, angle3):
    angle1 = angle1 + leg1_offset[0]
    angle2 = angle2 + leg1_offset[1]
    angle3 = angle3 + leg1_offset[2]

    while(channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3 ):
        if shutdown_event.is_set(): break

        ##ANGLE1
        if angle1 > channel_cur[0]:
            channel_cur[0] += 1
            setServo_invert_safe(0, channel_cur[0])
        elif angle1 < channel_cur[0]:
            channel_cur[0] -= 1
            setServo_invert_safe(0, channel_cur[0])

        ##ANGLE2
        if angle2 > channel_cur[1]:
            channel_cur[1] += 1
            setServo_invert_safe(1, channel_cur[1])
        elif angle2 < channel_cur[1]:
            channel_cur[1] -= 1
            setServo_invert_safe(1, channel_cur[1])

        ##ANGLE3
        if angle3 > channel_cur[2]:
            channel_cur[2] += 1
            setServo_safe(2, channel_cur[2])
        elif angle3 < channel_cur[2]:
            channel_cur[2] -= 1
            setServo_safe(2, channel_cur[2])

        time.sleep(move_delay)


def leg2(angle1, angle2, angle3):
    angle1 = angle1 + leg2_offset[0]
    angle2 = angle2 + leg2_offset[1]
    angle3 = angle3 + leg2_offset[2]

    while(channel_cur[3] != angle1 or channel_cur[4] != angle2 or channel_cur[5] != angle3 ):
        if shutdown_event.is_set(): break

        if angle1 > channel_cur[3]:
            channel_cur[3] += 1
            setServo_invert_safe(3, channel_cur[3])
        elif angle1 < channel_cur[3]:
            channel_cur[3] -= 1
            setServo_invert_safe(3, channel_cur[3])

        if angle2 > channel_cur[4]:
            channel_cur[4] += 1
            setServo_invert_safe(4, channel_cur[4])
        elif angle2 < channel_cur[4]:
            channel_cur[4] -= 1
            setServo_invert_safe(4, channel_cur[4])

        if angle3 > channel_cur[5]:
            channel_cur[5] += 1
            setServo_safe(5, channel_cur[5])
        elif angle3 < channel_cur[5]:
            channel_cur[5] -= 1
            setServo_safe(5, channel_cur[5])

        time.sleep(move_delay)


def leg3(angle1, angle2, angle3):
    angle1 = angle1 + leg3_offset[0]
    angle2 = angle2 + leg3_offset[1]
    angle3 = angle3 + leg3_offset[2]

    while(channel_cur[6] != angle1 or channel_cur[7] != angle2 or channel_cur[8] != angle3 ):
        if shutdown_event.is_set(): break

        if angle1 > channel_cur[6]:
            channel_cur[6] += 1
            setServo_safe(6, channel_cur[6])
        elif angle1 < channel_cur[6]:
            channel_cur[6] -= 1
            setServo_safe(6, channel_cur[6])

        if angle2 > channel_cur[7]:
            channel_cur[7] += 1
            setServo_invert_safe(7, channel_cur[7])
        elif angle2 < channel_cur[7]:
            channel_cur[7] -= 1
            setServo_invert_safe(7, channel_cur[7])

        if angle3 > channel_cur[8]:
            channel_cur[8] += 1
            setServo_safe(8, channel_cur[8])
        elif angle3 < channel_cur[8]:
            channel_cur[8] -= 1
            setServo_safe(8, channel_cur[8])

        time.sleep(move_delay)


def leg4(angle1, angle2, angle3):
    angle1 = angle1 + leg4_offset[0]
    angle2 = angle2 + leg4_offset[1]
    angle3 = angle3 + leg4_offset[2]

    while(channel_cur[9] != angle1 or channel_cur[10] != angle2 or channel_cur[11] != angle3 ):
        if shutdown_event.is_set(): break

        if angle1 > channel_cur[9]:
            channel_cur[9] += 1
            setServo_safe(9, channel_cur[9])
        elif angle1 < channel_cur[9]:
            channel_cur[9] -= 1
            setServo_safe(9, channel_cur[9])

        if angle2 > channel_cur[10]:
            channel_cur[10] += 1
            setServo_invert_safe(10, channel_cur[10])
        elif angle2 < channel_cur[10]:
            channel_cur[10] -= 1
            setServo_invert_safe(10, channel_cur[10])

        if angle3 > channel_cur[11]:
            channel_cur[11] += 1
            setServo_safe(11, channel_cur[11])
        elif angle3 < channel_cur[11]:
            channel_cur[11] -= 1
            setServo_safe(11, channel_cur[11])

        time.sleep(move_delay)


def begin():
    global leg_formation
    if shutdown_event.is_set(): return

    print("Beginning stance...", flush=True)
    
    # Move Left Side
    leg1(89,89,89)
    leg2(89,89,89)

    leg3(89,89,89)
    leg4(89,89,89)

    time.sleep(2)

    leg1(front_parallel,footdown,pincer_down)
    leg2(back_parallel,footdown,pincer_down)

    leg3(back_lateral,footdown,pincer_down)
    leg4(front_lateral,footdown,pincer_down)

    leg_formation = 1
    print("Ready.", flush=True)


# Updated Gait Functions with Reduced Concurrency (Max 2 threads)

def forward():
    global leg_formation
    if shutdown_event.is_set(): return

    if(leg_formation == 1):
        # lift leg1
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t2 = Thread(target=leg2, args=(back_lateral,footdown,pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral+back_lateral_add,footdown,pincer_down))
        
        t2.start()
        t3.start()
        t2.join()
        t3.join()
        
        leg4(front_parallel,footdown,pincer_down)

        # lift leg3
        leg3(back_lateral+back_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if (leg_formation == 2):
        # lift leg4
        leg4(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t3 = Thread(target=leg3, args=(back_lateral,footdown,pincer_down))
        t2 = Thread(target=leg2, args=(back_lateral+back_lateral_add,footdown,pincer_down))
        
        t3.start()
        t2.start()
        t3.join()
        t2.join()
        
        leg1(front_parallel,footdown,pincer_down)
        
        time.sleep(step_delay)

        # lift leg2
        leg2(back_lateral+back_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if(leg_formation == 1):
        leg_formation = 2
    elif(leg_formation == 2):
        leg_formation = 1


def backward():
    global leg_formation
    if shutdown_event.is_set(): return

    if(leg_formation == 1):
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t1 = Thread(target=leg1, args=(front_lateral,footdown,pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel,footdown,pincer_down))
        
        t1.start()
        t3.start()
        t1.join()
        t3.join()
        
        leg4(front_lateral+front_lateral_add,footdown,pincer_down)

        # lift leg4
        leg4(front_lateral+front_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if (leg_formation == 2):
        leg3(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t4 = Thread(target=leg4, args=(front_lateral,footdown,pincer_down))
        t2 = Thread(target=leg2, args=(back_parallel,footdown,pincer_down))
        
        t4.start()
        t2.start()
        t4.join()
        t2.join()
        
        leg1(front_lateral+front_lateral_add,footdown,pincer_down)

        time.sleep(step_delay)

        leg1(front_lateral+front_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if(leg_formation == 1):
        leg_formation = 2
    elif(leg_formation == 2):
        leg_formation = 1


def turn_left():
    global leg_formation
    if shutdown_event.is_set(): return

    if(leg_formation == 1):
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t1 = Thread(target=leg1, args=(front_lateral,footdown,pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral+back_lateral_add,footdown,pincer_down))
        
        t1.start()
        t3.start()
        t1.join()
        t3.join()
        
        leg4(front_parallel,footdown,pincer_down)

        leg2(back_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # lift leg3
        leg3(back_lateral+back_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if (leg_formation == 2):
        leg4(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t3 = Thread(target=leg3, args=(back_lateral,footdown,pincer_down))
        t2 = Thread(target=leg2, args=(back_parallel,footdown,pincer_down))
        
        t3.start()
        t2.start()
        t3.join()
        t2.join()
        
        leg1(front_lateral+front_lateral_add,footdown,pincer_down)

        leg4(front_lateral,footdown,pincer_down)

        time.sleep(step_delay)

        leg1(front_lateral+front_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if(leg_formation == 1):
        leg_formation = 2
    elif(leg_formation == 2):
        leg_formation = 1


def turn_right():
    global leg_formation
    if shutdown_event.is_set(): return

    if(leg_formation == 1):
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t2 = Thread(target=leg2, args=(back_lateral,footdown,pincer_down))
        t3 = Thread(target=leg3, args=(back_parallel,footdown,pincer_down))
        
        t2.start()
        t3.start()
        t2.join()
        t3.join()

        leg4(front_lateral+front_lateral_add,footdown,pincer_down)

        leg1(front_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # lift leg 4
        leg4(front_lateral+front_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg4(front_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if (leg_formation == 2):
        leg3(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg3(back_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        t1 = Thread(target=leg1, args=(front_parallel,footdown,pincer_down))
        t4 = Thread(target=leg4, args=(front_lateral,footdown,pincer_down))
        
        t1.start()
        t4.start()
        t1.join()
        t4.join()
        
        leg2(back_lateral+back_lateral_add,footdown,pincer_down)

        leg3(back_lateral,footdown,pincer_down)

        time.sleep(step_delay)

        leg2(back_lateral+back_lateral_add,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel,footdown,pincer_down)
        time.sleep(step_delay)

    if(leg_formation == 1):
        leg_formation = 2
    elif(leg_formation == 2):
        leg_formation = 1


def main():
    global leg_formation
    
    print_controls()
    begin()
    print("✅ Robot ready! Use arrow keys to control.\n", flush=True)
    
    try:
        while not shutdown_event.is_set():
            key = getch()
            
            if key == '\x1b':  # ESC sequence
                next1 = getch()
                if next1 == '[':  # Arrow key sequence
                    next2 = getch()
                    if next2 == 'A':  # UP arrow
                        print("→ Moving Forward", flush=True)
                        forward()
                    elif next2 == 'B':  # DOWN arrow
                        print("→ Moving Backward", flush=True)
                        backward()
                    elif next2 == 'D':  # LEFT arrow
                        print("→ Turning Left", flush=True)
                        turn_left()
                    elif next2 == 'C':  # RIGHT arrow
                        print("→ Turning Right", flush=True)
                        turn_right()
                else:  # ESC key alone
                    print("\nExiting...", flush=True)
                    break
            
            elif key == ' ':  # SPACE
                print("→ Stopping - Rest position", flush=True)
                begin()
            
            elif key == 'q' or key == 'Q':
                print("\nExiting...", flush=True)
                break
            
            elif key == '\x03':  # Ctrl+C
                break
    
    except KeyboardInterrupt:
        pass
    
    finally:
        print("\n🛑 Shutting down robot...", flush=True)
        try:
            for i in range(12):
                setServo_safe(i, 90)
        except:
            pass
        GPIO.cleanup()
        print("👋 Goodbye!", flush=True)


if __name__ == '__main__':
    main()
