#!/usr/bin/env python3
"""
Person Detection using YOLOv4-tiny
Optimized for Raspberry Pi 3B+ with Pi Camera 1.3
YOLOv4-tiny is lightweight and fast on Pi 3B+ (~8-12 FPS)
"""

from picamera2 import Picamera2
import cv2
import numpy as np
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import os

# Global variables
picam2 = None
frame_count = 0
start_time = None
latest_frame = None
frame_lock = threading.Lock()
person_count = 0
detection_fps = 0

# YOLOv4-tiny configuration
MODEL_DIR = "models"
WEIGHTS_PATH = os.path.join(MODEL_DIR, "yolov4-tiny.weights")
CONFIG_PATH = os.path.join(MODEL_DIR, "yolov4-tiny.cfg")
NAMES_PATH = os.path.join(MODEL_DIR, "coco.names")

# Detection parameters
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4  # Non-maximum suppression threshold
INPUT_SIZE = (416, 416)  # YOLOv4-tiny input size (416x416 is good balance)

# COCO dataset class names
CLASSES = []
if os.path.exists(NAMES_PATH):
    with open(NAMES_PATH, 'r') as f:
        CLASSES = [line.strip() for line in f.readlines()]
else:
    print(f"⚠️  {NAMES_PATH} not found. Will create it.")
    CLASSES = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
               "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
               "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
               "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
               "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
               "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
               "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
               "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
               "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
               "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

PERSON_CLASS_ID = 0  # "person" is class 0 in COCO

# Load YOLO model
print("Loading YOLOv4-tiny model...")
if not os.path.exists(WEIGHTS_PATH) or not os.path.exists(CONFIG_PATH):
    print("\n❌ Model files not found!")
    print("\n📥 Download YOLOv4-tiny model files:")
    print(f"   mkdir -p {MODEL_DIR}")
    print(f"   cd {MODEL_DIR}")
    print("   wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights")
    print("   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg")
    print("   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names")
    print(f"   cd ..")
    exit(1)

try:
    net = cv2.dnn.readNet(WEIGHTS_PATH, CONFIG_PATH)
    # Use CPU backend (Pi 3B+ doesn't have good GPU support for DNN)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
    # Get output layer names
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    
    print("✅ YOLOv4-tiny model loaded successfully!")
except Exception as e:
    print(f"❌ Error loading model: {e}")
    exit(1)


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head>
                <title>YOLOv4-tiny Person Detection</title>
                <style>
                    body { 
                        margin: 0; 
                        padding: 20px; 
                        background: #1a1a1a; 
                        color: white; 
                        font-family: Arial; 
                        text-align: center; 
                    }
                    img { 
                        max-width: 90%; 
                        border: 3px solid #FF6B35; 
                        border-radius: 8px;
                        box-shadow: 0 4px 6px rgba(0,0,0,0.3);
                    }
                    h1 { 
                        color: #FF6B35; 
                        margin-bottom: 10px;
                    }
                    .info {
                        background: #2a2a2a;
                        padding: 15px;
                        border-radius: 8px;
                        display: inline-block;
                        margin: 10px;
                    }
                    .status {
                        color: #FF6B35;
                        font-size: 18px;
                        font-weight: bold;
                    }
                    .detected {
                        color: #4CAF50;
                    }
                </style>
            </head>
            <body>
                <h1>🎯 YOLOv4-tiny Person Detection</h1>
                <div class="info">
                    <p class="status">Real-time Detection Active</p>
                    <p>Lightweight YOLO model optimized for Pi 3B+</p>
                    <p>Expected FPS: 8-12</p>
                </div>
                <br>
                <img src="/stream" />
                <br><br>
                <p>Press Ctrl+C in terminal to stop</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
        
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        if latest_frame is None:
                            continue
                        frame = latest_frame.copy()
                    
                    ret, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ret:
                        continue
                    
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                    time.sleep(0.033)
            except:
                pass
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass


def detect_persons_yolo(frame):
    """
    Detect persons using YOLOv4-tiny
    Returns: frame with bounding boxes, number of persons detected
    """
    global person_count, detection_fps
    
    detect_start = time.time()
    
    height, width = frame.shape[:2]
    
    # Create blob from image
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
    
    # Run detection
    net.setInput(blob)
    outputs = net.forward(output_layers)
    
    # Process detections
    boxes = []
    confidences = []
    class_ids = []
    
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            # Filter for person class with confidence threshold
            if class_id == PERSON_CLASS_ID and confidence > CONFIDENCE_THRESHOLD:
                # YOLO returns center x, center y, width, height (normalized)
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                
                # Calculate top-left corner
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
    
    # Apply Non-Maximum Suppression to remove overlapping boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    
    persons_detected = 0
    
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            confidence = confidences[i]
            persons_detected += 1
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw label with confidence
            label = f"Person: {confidence * 100:.1f}%"
            label_y = y - 10 if y - 10 > 10 else y + 10
            
            # Add background to label for better visibility
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(frame, (x, label_y - label_h - 5), (x + label_w, label_y + 5), (0, 255, 0), -1)
            cv2.putText(frame, label, (x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    person_count = persons_detected
    
    # Calculate detection FPS
    detection_fps = 1.0 / (time.time() - detect_start)
    
    return frame, persons_detected


def capture_and_detect():
    """Capture frames and run YOLO person detection"""
    global latest_frame, frame_count, start_time
    
    print("🎥 Starting YOLO detection loop...")
    
    while True:
        # Capture frame
        frame = picam2.capture_array()
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1
        elapsed = time.time() - start_time
        overall_fps = frame_count / elapsed if elapsed > 0 else 0
        
        # Run YOLO detection
        frame, persons = detect_persons_yolo(frame)
        
        # Add info overlay
        info_y = 30
        
        # Overall FPS (capture + detection)
        cv2.putText(frame, f"FPS: {overall_fps:.1f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        info_y += 35
        # Detection FPS (just YOLO inference)
        cv2.putText(frame, f"Detection: {detection_fps:.1f} FPS", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        info_y += 35
        # Person count
        color = (0, 255, 0) if persons > 0 else (255, 255, 255)
        cv2.putText(frame, f"Persons: {persons}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        info_y += 35
        cv2.putText(frame, f"Frame: {frame_count}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Detection status indicator (top-right)
        if persons > 0:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 15, (0, 255, 0), -1)
            cv2.putText(frame, "DETECTED", (frame.shape[1] - 150, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 15, (128, 128, 128), 2)
        
        with frame_lock:
            latest_frame = frame
        
        # Small delay to prevent CPU overload
        time.sleep(0.01)


def main():
    global picam2, start_time
    
    print("\n" + "="*60)
    print("🎯 PERSON DETECTION - YOLOv4-tiny")
    print("="*60)
    print("Optimized for Raspberry Pi 3B+")
    print("="*60 + "\n")
    
    # Initialize camera
    print("📷 Initializing Pi Camera...")
    picam2 = Picamera2()
    
    # Use 640x480 for good balance (YOLO will resize to 416x416 internally)
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    print("✅ Camera initialized!")
    
    start_time = time.time()
    
    # Start detection thread
    detection_thread = threading.Thread(target=capture_and_detect, daemon=True)
    detection_thread.start()
    print("✅ Detection thread started!")
    
    # Start HTTP server
    try:
        server = ThreadedHTTPServer(('0.0.0.0', 8000), StreamingHandler)
        print("\n" + "="*60)
        print("🌐 HTTP Server running!")
        print("="*60)
        print("Access the stream at:")
        print("  http://<raspberry-pi-ip>:8000")
        print("\nTo find your Pi's IP: hostname -I")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping server...")
    finally:
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0
        picam2.stop()
        print(f"\n📊 Statistics:")
        print(f"   Total frames: {frame_count}")
        print(f"   Runtime: {total_time:.2f}s")
        print(f"   Average FPS: {avg_fps:.2f}")
        print(f"   Detection FPS: {detection_fps:.2f}")
        print("\n👋 Goodbye!")


if __name__ == "__main__":
    main()
