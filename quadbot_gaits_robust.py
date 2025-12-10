#!/usr/bin/env python3
"""
Robust quadbot gait control using adafruit-circuitpython-pca9685
Modern CircuitPython library version with all safety features
"""
from __future__ import division
import time
import signal
import sys
import RPi.GPIO as GPIO
from threading import Thread, Lock, Event
import board
import busio
from adafruit_pca9685 import PCA9685

# Global shutdown event for clean exit
shutdown_event = Event()

cur_angle_mutex = Lock()
i2c_mutex = Lock()

# GPIO pin definitions for leg sensors
leg1_s = 37
leg2_s = 35
leg3_s = 33
leg4_s = 31

# Initialize I2C bus and PCA9685
print("Initializing PCA9685...")
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# Servo pulse range (in microseconds for CircuitPython library)
# These values map to the original 150-600 PWM range
SERVO_MIN_PULSE = 500   # Minimum pulse width in microseconds
SERVO_MAX_PULSE = 2500  # Maximum pulse width in microseconds

# FIXED: Increased delays to prevent Pi freeze
move_delay = 0.01  # Changed from 0.0005 to 0.01 (10ms)
step_delay = 0.05  # Changed from 0.001 to 0.05 (50ms)

# Servo step size - move multiple degrees at once to reduce I2C traffic
servo_step = 2  # Changed from 1 to 2 degrees per update

# Maximum iterations to prevent infinite loops
MAX_ITERATIONS = 200

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

channel_cur = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Interrupt received! Shutting down...")
    shutdown_event.set()
    try:
        # Return servos to neutral (90 degrees)
        for i in range(12):
            angle_to_pulse_width(90, i)
            time.sleep(0.01)
    except:
        pass
    pca.deinit()
    GPIO.cleanup()
    sys.exit(0)


# Register signal handler
signal.signal(signal.SIGINT, signal_handler)


def pinsetup():
    """Setup GPIO pins"""
    GPIO.setwarnings(False)
    try:
        GPIO.setmode(GPIO.BOARD)
    except ValueError:
        pass  # Mode already set
    
    GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def clamp_angle(angle):
    """Ensure angle is within valid servo range"""
    return max(0, min(180, int(angle)))


def angle_to_pulse_width(angle, channel):
    """
    Convert angle (0-180) to pulse width and set servo
    This matches the original PWM formula: pwm_value = (angle * 2.5) + 150
    """
    # Map 0-180 degrees to pulse width in microseconds
    # Original formula: pwm = (angle * 2.5) + 150 (out of 4096 at 60Hz)
    # Convert to microseconds: (pwm / 4096) * (1000000 / 60)
    pulse_range = SERVO_MAX_PULSE - SERVO_MIN_PULSE
    pulse_width = SERVO_MIN_PULSE + (angle / 180.0) * pulse_range
    
    i2c_mutex.acquire()
    try:
        pca.channels[channel].duty_cycle = int((pulse_width / 1000000.0) * 65535 * pca.frequency)
        time.sleep(0.002)  # Small delay after I2C write
    except Exception as e:
        print(f"⚠️  Error setting servo {channel}: {e}")
    finally:
        i2c_mutex.release()


def setServo(channel, angle):
    """Set servo using angle (0-180)"""
    if shutdown_event.is_set():
        return
    
    angle = clamp_angle(angle)
    angle_to_pulse_width(angle, channel)


def setServo_invert(channel, angle):
    """Set inverted servo using angle (0-180)"""
    if shutdown_event.is_set():
        return
    
    angle = clamp_angle(angle)
    inverted_angle = 180 - angle
    angle_to_pulse_width(inverted_angle, channel)


def leg1(angle1, angle2, angle3):
    """Control leg 1 with timeout protection"""
    angle1 = clamp_angle(angle1 + leg1_offset[0])
    angle2 = clamp_angle(angle2 + leg1_offset[1])
    angle3 = clamp_angle(angle3 + leg1_offset[2])
    
    iterations = 0
    while (abs(channel_cur[0] - angle1) > 1 or 
           abs(channel_cur[1] - angle2) > 1 or 
           abs(channel_cur[2] - angle3) > 1):
        
        if shutdown_event.is_set() or iterations >= MAX_ITERATIONS:
            if iterations >= MAX_ITERATIONS:
                print(f"⚠️  Leg1 timeout: target=({angle1},{angle2},{angle3}) current=({channel_cur[0]},{channel_cur[1]},{channel_cur[2]})")
            break
        
        # ANGLE1
        if angle1 > channel_cur[0]:
            channel_cur[0] = min(channel_cur[0] + servo_step, angle1)
            setServo_invert(0, channel_cur[0])
        elif angle1 < channel_cur[0]:
            channel_cur[0] = max(channel_cur[0] - servo_step, angle1)
            setServo_invert(0, channel_cur[0])
        
        # ANGLE2
        if angle2 > channel_cur[1]:
            channel_cur[1] = min(channel_cur[1] + servo_step, angle2)
            setServo_invert(1, channel_cur[1])
        elif angle2 < channel_cur[1]:
            channel_cur[1] = max(channel_cur[1] - servo_step, angle2)
            setServo_invert(1, channel_cur[1])
        
        # ANGLE3
        if angle3 > channel_cur[2]:
            channel_cur[2] = min(channel_cur[2] + servo_step, angle3)
            setServo(2, channel_cur[2])
        elif angle3 < channel_cur[2]:
            channel_cur[2] = max(channel_cur[2] - servo_step, angle3)
            setServo(2, channel_cur[2])
        
        time.sleep(move_delay)
        iterations += 1


def leg2(angle1, angle2, angle3):
    """Control leg 2 with timeout protection"""
    angle1 = clamp_angle(angle1 + leg2_offset[0])
    angle2 = clamp_angle(angle2 + leg2_offset[1])
    angle3 = clamp_angle(angle3 + leg2_offset[2])
    
    iterations = 0
    while (abs(channel_cur[3] - angle1) > 1 or 
           abs(channel_cur[4] - angle2) > 1 or 
           abs(channel_cur[5] - angle3) > 1):
        
        if shutdown_event.is_set() or iterations >= MAX_ITERATIONS:
            if iterations >= MAX_ITERATIONS:
                print(f"⚠️  Leg2 timeout: target=({angle1},{angle2},{angle3}) current=({channel_cur[3]},{channel_cur[4]},{channel_cur[5]})")
            break
        
        # ANGLE1
        if angle1 > channel_cur[3]:
            channel_cur[3] = min(channel_cur[3] + servo_step, angle1)
            setServo_invert(3, channel_cur[3])
        elif angle1 < channel_cur[3]:
            channel_cur[3] = max(channel_cur[3] - servo_step, angle1)
            setServo_invert(3, channel_cur[3])
        
        # ANGLE2
        if angle2 > channel_cur[4]:
            channel_cur[4] = min(channel_cur[4] + servo_step, angle2)
            setServo_invert(4, channel_cur[4])
        elif angle2 < channel_cur[4]:
            channel_cur[4] = max(channel_cur[4] - servo_step, angle2)
            setServo_invert(4, channel_cur[4])
        
        # ANGLE3
        if angle3 > channel_cur[5]:
            channel_cur[5] = min(channel_cur[5] + servo_step, angle3)
            setServo(5, channel_cur[5])
        elif angle3 < channel_cur[5]:
            channel_cur[5] = max(channel_cur[5] - servo_step, angle3)
            setServo(5, channel_cur[5])
        
        time.sleep(move_delay)
        iterations += 1


def leg3(angle1, angle2, angle3):
    """Control leg 3 with timeout protection"""
    angle1 = clamp_angle(angle1 + leg3_offset[0])
    angle2 = clamp_angle(angle2 + leg3_offset[1])
    angle3 = clamp_angle(angle3 + leg3_offset[2])
    
    iterations = 0
    while (abs(channel_cur[6] - angle1) > 1 or 
           abs(channel_cur[7] - angle2) > 1 or 
           abs(channel_cur[8] - angle3) > 1):
        
        if shutdown_event.is_set() or iterations >= MAX_ITERATIONS:
            if iterations >= MAX_ITERATIONS:
                print(f"⚠️  Leg3 timeout: target=({angle1},{angle2},{angle3}) current=({channel_cur[6]},{channel_cur[7]},{channel_cur[8]})")
            break
        
        # ANGLE1
        if angle1 > channel_cur[6]:
            channel_cur[6] = min(channel_cur[6] + servo_step, angle1)
            setServo(6, channel_cur[6])
        elif angle1 < channel_cur[6]:
            channel_cur[6] = max(channel_cur[6] - servo_step, angle1)
            setServo(6, channel_cur[6])
        
        # ANGLE2
        if angle2 > channel_cur[7]:
            channel_cur[7] = min(channel_cur[7] + servo_step, angle2)
            setServo_invert(7, channel_cur[7])
        elif angle2 < channel_cur[7]:
            channel_cur[7] = max(channel_cur[7] - servo_step, angle2)
            setServo_invert(7, channel_cur[7])
        
        # ANGLE3
        if angle3 > channel_cur[8]:
            channel_cur[8] = min(channel_cur[8] + servo_step, angle3)
            setServo(8, channel_cur[8])
        elif angle3 < channel_cur[8]:
            channel_cur[8] = max(channel_cur[8] - servo_step, angle3)
            setServo(8, channel_cur[8])
        
        time.sleep(move_delay)
        iterations += 1


def leg4(angle1, angle2, angle3):
    """Control leg 4 with timeout protection"""
    angle1 = clamp_angle(angle1 + leg4_offset[0])
    angle2 = clamp_angle(angle2 + leg4_offset[1])
    angle3 = clamp_angle(angle3 + leg4_offset[2])
    
    iterations = 0
    while (abs(channel_cur[9] - angle1) > 1 or 
           abs(channel_cur[10] - angle2) > 1 or 
           abs(channel_cur[11] - angle3) > 1):
        
        if shutdown_event.is_set() or iterations >= MAX_ITERATIONS:
            if iterations >= MAX_ITERATIONS:
                print(f"⚠️  Leg4 timeout: target=({angle1},{angle2},{angle3}) current=({channel_cur[9]},{channel_cur[10]},{channel_cur[11]})")
            break
        
        # ANGLE1
        if angle1 > channel_cur[9]:
            channel_cur[9] = min(channel_cur[9] + servo_step, angle1)
            setServo(9, channel_cur[9])
        elif angle1 < channel_cur[9]:
            channel_cur[9] = max(channel_cur[9] - servo_step, angle1)
            setServo(9, channel_cur[9])
        
        # ANGLE2
        if angle2 > channel_cur[10]:
            channel_cur[10] = min(channel_cur[10] + servo_step, angle2)
            setServo_invert(10, channel_cur[10])
        elif angle2 < channel_cur[10]:
            channel_cur[10] = max(channel_cur[10] - servo_step, angle2)
            setServo_invert(10, channel_cur[10])
        
        # ANGLE3
        if angle3 > channel_cur[11]:
            channel_cur[11] = min(channel_cur[11] + servo_step, angle3)
            setServo(11, channel_cur[11])
        elif angle3 < channel_cur[11]:
            channel_cur[11] = max(channel_cur[11] - servo_step, angle3)
            setServo(11, channel_cur[11])
        
        time.sleep(move_delay)
        iterations += 1


def begin():
    """Initialize robot to standing position"""
    global leg_formation
    
    print("🤖 Moving to initial position...")
    # Move legs one at a time to avoid overload
    leg1(89, 89, 89)
    time.sleep(0.3)
    leg2(89, 89, 89)
    time.sleep(0.3)
    leg3(89, 89, 89)
    time.sleep(0.3)
    leg4(89, 89, 89)
    time.sleep(1)
    
    print("🤖 Moving to stance position...")
    leg1(front_parallel, footdown, pincer_down)
    time.sleep(0.3)
    leg2(back_parallel, footdown, pincer_down)
    time.sleep(0.3)
    leg3(back_lateral, footdown, pincer_down)
    time.sleep(0.3)
    leg4(front_lateral, footdown, pincer_down)
    time.sleep(0.5)
    
    leg_formation = 1
    print("✅ Robot ready!")


def turn_right():
    """Turn robot right"""
    global leg_formation
    if shutdown_event.is_set():
        return
    
    print("↻ Turn Right")
    
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
        
        leg2(back_lateral + back_lateral_add, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footup, pincer_up)
        time.sleep(step_delay)
        leg2(back_parallel, footdown, pincer_down)
        time.sleep(step_delay)
    
    leg_formation = 2 if leg_formation == 1 else 1


def main():
    """Main program"""
    print("\n" + "="*60)
    print("🤖 QUADBOT GAIT CONTROL - CircuitPython PCA9685")
    print("="*60)
    print("Using modern adafruit-circuitpython-pca9685 library")
    print("Press Ctrl+C to stop at any time")
    print("="*60 + "\n")
    
    try:
        pinsetup()
        print("✅ GPIO pins configured!")
        print("✅ PCA9685 initialized!")
        
        begin()
        time.sleep(1)
        
        print("\n🎯 Executing 4 right turns...")
        for i in range(4):
            if shutdown_event.is_set():
                break
            print(f"\nTurn {i+1}/4")
            turn_right()
            time.sleep(0.5)
        
        print("\n✅ Program completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Returning to neutral position...")
        for i in range(12):
            try:
                angle_to_pulse_width(90, i)
                time.sleep(0.01)
            except:
                pass
        pca.deinit()
        GPIO.cleanup()
        print("👋 Goodbye!")


if __name__ == '__main__':
    main()
