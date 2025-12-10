from __future__ import division
import time
import RPi.GPIO as GPIO
from threading import Thread, Lock
from adafruit_servokit import ServoKit
import sys

# Locks
cur_angle_mutex = Lock()
i2c_mutex = Lock()

# GPIO pin definitions for leg sensors
leg1_s = 37
leg2_s = 35
leg3_s = 33
leg4_s = 31

# Setup GPIO
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass
GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize ServoKit
print("Initializing ServoKit...")
kit = ServoKit(channels=16)

# Set actuation range
for i in range(16):
    kit.servo[i].actuation_range = 180

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

leg1_footdown = footdown
leg2_footdown = footdown
leg3_footdown = footdown
leg4_footdown = footdown

leg_formation = 0

channel_cur = [90]*12

def setServo(channel, angle):
    if angle < 0: angle = 0
    elif angle > 180: angle = 180
    
    with i2c_mutex:
        try:
            kit.servo[channel].angle = angle
        except Exception as e:
            print(f"Error setting servo {channel}: {e}", flush=True)

def setServo_invert(channel, angle):
    if angle < 0: angle = 0
    elif angle > 180: angle = 180

    with i2c_mutex:
        try:
            # Invert: 0->180, 180->0
            kit.servo[channel].angle = 180 - angle
        except Exception as e:
            print(f"Error setting servo {channel}: {e}", flush=True)

def leg1(angle1,angle2,angle3):
    angle1 = angle1+leg1_offset[0]
    angle2 = angle2+leg1_offset[1]
    angle3 = angle3+leg1_offset[2]

    # Simple approach: Loop until all angles reached
    while(channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3 ):
        ##ANGLE1
        if angle1 > channel_cur[0]:
            channel_cur[0] += 1
            setServo_invert(0,channel_cur[0])
        elif angle1 < channel_cur[0]:
            channel_cur[0] -= 1
            setServo_invert(0,channel_cur[0])

        ##ANGLE2
        if angle2 > channel_cur[1]:
            channel_cur[1] += 1
            setServo_invert(1,channel_cur[1])
        elif angle2 < channel_cur[1]:
            channel_cur[1] -= 1
            setServo_invert(1,channel_cur[1])

        ##ANGLE3
        if angle3 > channel_cur[2]:
            channel_cur[2] += 1
            setServo(2,channel_cur[2])
        elif angle3 < channel_cur[2]:
            channel_cur[2] -= 1
            setServo(2,channel_cur[2])

        time.sleep(move_delay)

def leg2(angle1,angle2,angle3):
    angle1 = angle1+leg2_offset[0]
    angle2 = angle2+leg2_offset[1]
    angle3 = angle3+leg2_offset[2]

    while(channel_cur[3] != angle1 or channel_cur[4] != angle2 or channel_cur[5] != angle3 ):
        if angle1 > channel_cur[3]:
            channel_cur[3] += 1
            setServo_invert(3,channel_cur[3])
        elif angle1 < channel_cur[3]:
            channel_cur[3] -= 1
            setServo_invert(3,channel_cur[3])

        if angle2 > channel_cur[4]:
            channel_cur[4] += 1
            setServo_invert(4,channel_cur[4])
        elif angle2 < channel_cur[4]:
            channel_cur[4] -= 1
            setServo_invert(4,channel_cur[4])

        if angle3 > channel_cur[5]:
            channel_cur[5] += 1
            setServo(5,channel_cur[5])
        elif angle3 < channel_cur[5]:
            channel_cur[5] -= 1
            setServo(5,channel_cur[5])

        time.sleep(move_delay)

def leg3(angle1,angle2,angle3):
    angle1 = angle1+leg3_offset[0]
    angle2 = angle2+leg3_offset[1]
    angle3 = angle3+leg3_offset[2]

    while(channel_cur[6] != angle1 or channel_cur[7] != angle2 or channel_cur[8] != angle3 ):
        if angle1 > channel_cur[6]:
            channel_cur[6] += 1
            setServo(6,channel_cur[6])
        elif angle1 < channel_cur[6]:
            channel_cur[6] -= 1
            setServo(6,channel_cur[6])

        if angle2 > channel_cur[7]:
            channel_cur[7] += 1
            setServo_invert(7,channel_cur[7])
        elif angle2 < channel_cur[7]:
            channel_cur[7] -= 1
            setServo_invert(7,channel_cur[7])

        if angle3 > channel_cur[8]:
            channel_cur[8] += 1
            setServo(8,channel_cur[8])
        elif angle3 < channel_cur[8]:
            channel_cur[8] -= 1
            setServo(8,channel_cur[8])

        time.sleep(move_delay)

def leg4(angle1,angle2,angle3):
    angle1 = angle1+leg4_offset[0]
    angle2 = angle2+leg4_offset[1]
    angle3 = angle3+leg4_offset[2]

    while(channel_cur[9] != angle1 or channel_cur[10] != angle2 or channel_cur[11] != angle3 ):
        if angle1 > channel_cur[9]:
            channel_cur[9] += 1
            setServo(9,channel_cur[9])
        elif angle1 < channel_cur[9]:
            channel_cur[9] -= 1
            setServo(9,channel_cur[9])

        if angle2 > channel_cur[10]:
            channel_cur[10] += 1
            setServo_invert(10,channel_cur[10])
        elif angle2 < channel_cur[10]:
            channel_cur[10] -= 1
            setServo_invert(10,channel_cur[10])

        if angle3 > channel_cur[11]:
            channel_cur[11] += 1
            setServo(11,channel_cur[11])
        elif angle3 < channel_cur[11]:
            channel_cur[11] -= 1
            setServo(11,channel_cur[11])

        time.sleep(move_delay)

def begin():
    global leg_formation
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

def forward():
    global leg_formation
    if(leg_formation == 1):
        # lift leg1
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY TO 2 THREADS
        # Originally: t2, t3, t4. Now: t2+t3, then t4
        t2 = Thread(target=leg2, args=(back_lateral,footdown,pincer_down))
        t3 = Thread(target=leg3, args=(back_lateral+back_lateral_add,footdown,pincer_down))
        
        t2.start()
        t3.start()
        t2.join()
        t3.join()
        
        # Run leg4 separately
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

        # REDUCED CONCURRENCY TO 2 THREADS
        # Originally: t3, t2, t1. Now t3+t2, then t1.
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
    if(leg_formation == 1):
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footdown,pincer_down)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        # Originally t1, t3, t4. Now t1+t3, then t4
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
        # Originally t4, t2, t1. Now t4+t2, then t1
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
    if(leg_formation == 1):
        leg2(back_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg2(back_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        # Originally t1, t3, t4. Now t1+t3, then t4.
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
        # Originally t3, t2, t1. Now t3+t2, then t1
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
    if(leg_formation == 1):
        leg1(front_parallel,footup,pincer_up)
        time.sleep(step_delay)
        leg1(front_lateral,footup,pincer_up)
        time.sleep(step_delay)

        # REDUCED CONCURRENCY
        # Originally t2, t3, t4. Now t2+t3, then t4
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
        # Originally t1, t4, t2. Now t1+t4, then t2.
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
    print("Starting Main...", flush=True)
    begin()
    time.sleep(1)

    print("Turning right 4x...", flush=True)
    turn_right()
    turn_right()
    turn_right()
    turn_right()
    
    # for x in range(0,5):
    #     forward()

    # for x in range(0,15):
    #     for x in range(0,15):
    #     turn_left()

    print("Done.", flush=True)

if __name__ == '__main__':
    main()
