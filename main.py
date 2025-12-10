#!/usr/bin/env python3
"""
Quadbot Web Controller
Combines video streaming and robot control logic
"""
from flask import Flask, render_template, Response, jsonify
import cv2
import threading
import time
from threading import Thread, Lock
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from picamera2 import Picamera2

# --- Logic from quadbot_gaits_robust.py ---
# Includes robust movement and thread safety

# GPIO & Servo Config
leg1_s, leg2_s, leg3_s, leg4_s = 37, 35, 33, 31
GPIO.setwarnings(False)
try:
    GPIO.setmode(GPIO.BOARD)
except ValueError:
    pass
GPIO.setup(leg1_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg2_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg3_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leg4_s, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Initializing ServoKit...", flush=True)
kit = ServoKit(channels=16)
for i in range(16):
    kit.servo[i].actuation_range = 180

# State Variables
cur_angle_mutex = Lock()
i2c_mutex = Lock()
robot_busy = Lock() # Prevent overlapping commands

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
channel_cur = [90]*12

# CAMERA GLOBALS
picam2 = None
latest_frame = None
frame_lock = threading.Lock()

# --- SERVO FUNCTIONS ---

def setServo_safe(channel, angle):
    if angle < 0: angle = 0
    elif angle > 180: angle = 180
    with i2c_mutex:
        try:
            kit.servo[channel].angle = angle
        except: pass

def setServo_invert_safe(channel, angle):
    if angle < 0: angle = 0
    elif angle > 180: angle = 180
    with i2c_mutex:
        try:
            kit.servo[channel].angle = 180 - angle
        except: pass

def leg1(angle1, angle2, angle3):
    angle1 += leg1_offset[0]
    angle2 += leg1_offset[1]
    angle3 += leg1_offset[2]
    
    while(channel_cur[0] != angle1 or channel_cur[1] != angle2 or channel_cur[2] != angle3):
        if angle1 > channel_cur[0]:
            channel_cur[0] += 1
            setServo_invert_safe(0, channel_cur[0])
        elif angle1 < channel_cur[0]:
            channel_cur[0] -= 1
            setServo_invert_safe(0, channel_cur[0])

        if angle2 > channel_cur[1]:
            channel_cur[1] += 1
            setServo_invert_safe(1, channel_cur[1])
        elif angle2 < channel_cur[1]:
            channel_cur[1] -= 1
            setServo_invert_safe(1, channel_cur[1])

        if angle3 > channel_cur[2]:
            channel_cur[2] += 1
            setServo_safe(2, channel_cur[2])
        elif angle3 < channel_cur[2]:
            channel_cur[2] -= 1
            setServo_safe(2, channel_cur[2])
        
        time.sleep(move_delay)

def leg2(angle1, angle2, angle3):
    angle1 += leg2_offset[0]
    angle2 += leg2_offset[1]
    angle3 += leg2_offset[2]
    
    while(channel_cur[3] != angle1 or channel_cur[4] != angle2 or channel_cur[5] != angle3):
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
    angle1 += leg3_offset[0]
    angle2 += leg3_offset[1]
    angle3 += leg3_offset[2]
    
    while(channel_cur[6] != angle1 or channel_cur[7] != angle2 or channel_cur[8] != angle3):
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
    angle1 += leg4_offset[0]
    angle2 += leg4_offset[1]
    angle3 += leg4_offset[2]
    
    while(channel_cur[9] != angle1 or channel_cur[10] != angle2 or channel_cur[11] != angle3):
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

# --- GAIT FUNCTIONS (ROBUST VERSION) ---
def begin_pose():
    global leg_formation
    if robot_busy.acquire(blocking=False):
        try:
            leg1(89,89,89); leg2(89,89,89); leg3(89,89,89); leg4(89,89,89)
            time.sleep(1)
            leg1(front_parallel,footdown,pincer_down)
            leg2(back_parallel,footdown,pincer_down)
            leg3(back_lateral,footdown,pincer_down)
            leg4(front_lateral,footdown,pincer_down)
            leg_formation = 1
        finally:
            robot_busy.release()

def move_forward():
    global leg_formation
    if not robot_busy.acquire(blocking=False): return
    try:
        if(leg_formation == 1):
            leg1(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg1(front_lateral,footup,pincer_up); time.sleep(step_delay)
            leg1(front_lateral,footdown,pincer_down); time.sleep(step_delay)

            t2 = Thread(target=leg2, args=(back_lateral,footdown,pincer_down))
            t3 = Thread(target=leg3, args=(back_lateral+back_lateral_add,footdown,pincer_down))
            t2.start(); t3.start(); t2.join(); t3.join()
            
            leg4(front_parallel,footdown,pincer_down)

            leg3(back_lateral+back_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg3(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg3(back_parallel,footdown,pincer_down); time.sleep(step_delay)

        if(leg_formation == 2):
            leg4(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg4(front_lateral,footup,pincer_up); time.sleep(step_delay)
            leg4(front_lateral,footdown,pincer_down); time.sleep(step_delay)

            t3 = Thread(target=leg3, args=(back_lateral,footdown,pincer_down))
            t2 = Thread(target=leg2, args=(back_lateral+back_lateral_add,footdown,pincer_down))
            t3.start(); t2.start(); t3.join(); t2.join()
            
            leg1(front_parallel,footdown,pincer_down); time.sleep(step_delay)

            leg2(back_lateral+back_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg2(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg2(back_parallel,footdown,pincer_down); time.sleep(step_delay)

        leg_formation = 2 if leg_formation == 1 else 1
    finally:
        robot_busy.release()

def move_backward():
    global leg_formation
    if not robot_busy.acquire(blocking=False): return
    try:
        if(leg_formation == 1):
            leg2(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg2(back_lateral,footup,pincer_up); time.sleep(step_delay)
            leg2(back_lateral,footdown,pincer_down); time.sleep(step_delay)

            t1 = Thread(target=leg1, args=(front_lateral,footdown,pincer_down))
            t3 = Thread(target=leg3, args=(back_parallel,footdown,pincer_down))
            t1.start(); t3.start(); t1.join(); t3.join()
            
            leg4(front_lateral+front_lateral_add,footdown,pincer_down)

            leg4(front_lateral+front_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg4(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg4(front_parallel,footdown,pincer_down); time.sleep(step_delay)

        if(leg_formation == 2):
            leg3(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg3(back_lateral,footup,pincer_up); time.sleep(step_delay)
            leg3(back_lateral,footdown,pincer_down); time.sleep(step_delay)

            t4 = Thread(target=leg4, args=(front_lateral,footdown,pincer_down))
            t2 = Thread(target=leg2, args=(back_parallel,footdown,pincer_down))
            t4.start(); t2.start(); t4.join(); t2.join()
            
            leg1(front_lateral+front_lateral_add,footdown,pincer_down); time.sleep(step_delay)

            leg1(front_lateral+front_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg1(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg1(front_parallel,footdown,pincer_down); time.sleep(step_delay)

        leg_formation = 2 if leg_formation == 1 else 1
    finally:
        robot_busy.release()

def turn_left():
    global leg_formation
    if not robot_busy.acquire(blocking=False): return
    try:
        if(leg_formation == 1):
            leg2(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg2(back_lateral,footup,pincer_up); time.sleep(step_delay)

            t1 = Thread(target=leg1, args=(front_lateral,footdown,pincer_down))
            t3 = Thread(target=leg3, args=(back_lateral+back_lateral_add,footdown,pincer_down))
            t1.start(); t3.start(); t1.join(); t3.join()
            
            leg4(front_parallel,footdown,pincer_down)
            leg2(back_lateral,footdown,pincer_down); time.sleep(step_delay)

            leg3(back_lateral+back_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg3(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg3(back_parallel,footdown,pincer_down); time.sleep(step_delay)

        if(leg_formation == 2):
            leg4(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg4(front_lateral,footup,pincer_up); time.sleep(step_delay)

            t3 = Thread(target=leg3, args=(back_lateral,footdown,pincer_down))
            t2 = Thread(target=leg2, args=(back_parallel,footdown,pincer_down))
            t3.start(); t2.start(); t3.join(); t2.join()
            
            leg1(front_lateral+front_lateral_add,footdown,pincer_down)
            leg4(front_lateral,footdown,pincer_down); time.sleep(step_delay)

            leg1(front_lateral+front_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg1(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg1(front_parallel,footdown,pincer_down); time.sleep(step_delay)

        leg_formation = 2 if leg_formation == 1 else 1
    finally:
        robot_busy.release()

def turn_right():
    global leg_formation
    if not robot_busy.acquire(blocking=False): return
    try:
        if(leg_formation == 1):
            leg1(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg1(front_lateral,footup,pincer_up); time.sleep(step_delay)

            t2 = Thread(target=leg2, args=(back_lateral,footdown,pincer_down))
            t3 = Thread(target=leg3, args=(back_parallel,footdown,pincer_down))
            t2.start(); t3.start(); t2.join(); t3.join()
            
            leg4(front_lateral+front_lateral_add,footdown,pincer_down)
            leg1(front_lateral,footdown,pincer_down); time.sleep(step_delay)

            leg4(front_lateral+front_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg4(front_parallel,footup,pincer_up); time.sleep(step_delay)
            leg4(front_parallel,footdown,pincer_down); time.sleep(step_delay)

        if(leg_formation == 2):
            leg3(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg3(back_lateral,footup,pincer_up); time.sleep(step_delay)

            t1 = Thread(target=leg1, args=(front_parallel,footdown,pincer_down))
            t4 = Thread(target=leg4, args=(front_lateral,footdown,pincer_down))
            t1.start(); t4.start(); t1.join(); t4.join()
            
            leg2(back_lateral+back_lateral_add,footdown,pincer_down)
            leg3(back_lateral,footdown,pincer_down); time.sleep(step_delay)

            leg2(back_lateral+back_lateral_add,footup,pincer_up); time.sleep(step_delay)
            leg2(back_parallel,footup,pincer_up); time.sleep(step_delay)
            leg2(back_parallel,footdown,pincer_down); time.sleep(step_delay)

        leg_formation = 2 if leg_formation == 1 else 1
    finally:
        robot_busy.release()

# --- FLASK APP ---

app = Flask(__name__)

def capture_frames():
    global latest_frame
    while True:
        try:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with frame_lock:
                latest_frame = frame
        except Exception as e:
            print(f"Cam Error: {e}")
            time.sleep(1)

def generate_frames():
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
        
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control/<cmd>')
def control(cmd):
    if robot_busy.locked():
        return jsonify(status="busy")
    
    t = None
    if cmd == 'forward': t = Thread(target=move_forward)
    elif cmd == 'backward': t = Thread(target=move_backward)
    elif cmd == 'left': t = Thread(target=turn_left)
    elif cmd == 'right': t = Thread(target=turn_right)
    elif cmd == 'stop': t = Thread(target=begin_pose)
    
    if t:
        t.start()
        return jsonify(status="ok", cmd=cmd)
    
    return jsonify(status="invalid")

if __name__ == '__main__':
    # Init Camera
    print("Starting Camera...", flush=True)
    picam2 = Picamera2()
    # config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    # Correct config creation for headless
    config = picam2.create_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    
    # Cam Thread
    cam_thread = Thread(target=capture_frames, daemon=True)
    cam_thread.start()
    
    # Move to initial pose
    begin_pose()
    
    # Start Server
    print("Starting Web Server...", flush=True)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
