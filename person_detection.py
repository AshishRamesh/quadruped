#!/usr/bin/env python3
"""
Person Detection using Pi Camera 1.3 on Raspberry Pi 3B+
Uses MobileNet SSD for efficient person detection
Streams results over HTTP at http://<pi-ip>:8000
"""

from picamera2 import Picamera2
import cv2
import numpy as np
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading

# Global variables
picam2 = None
frame_count = 0
start_time = None
latest_frame = None
frame_lock = threading.Lock()
detection_enabled = True
person_count = 0
last_detection_time = 0

# MobileNet SSD configuration
# Download model files first (see instructions below)
MODEL_PATH = "models/MobileNetSSD_deploy.caffemodel"
PROTOTXT_PATH = "models/MobileNetSSD_deploy.prototxt"

# MobileNet SSD class labels
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

# Detection parameters
CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence for detection
INPUT_SIZE = (300, 300)     # Model input size (smaller = faster)
PERSON_CLASS_ID = 15        # Index for "person" in CLASSES

# Load the model
print("Loading MobileNet SSD model...")
try:
    net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATH, MODEL_PATH)
    print("✅ Model loaded successfully!")
except Exception as e:
    print(f"❌ Error loading model: {e}")
    print("\n📥 Please download the model files:")
    print("   mkdir -p models")
    print("   cd models")
    print("   wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.prototxt")
    print("   wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.caffemodel")
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
                <title>Person Detection - Pi Camera</title>
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
                        border: 3px solid #4CAF50; 
                        border-radius: 8px;
                    }
                    h1 { 
                        color: #4CAF50; 
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
                        color: #4CAF50;
                        font-size: 18px;
                        font-weight: bold;
                    }
                    button {
                        background: #4CAF50;
                        color: white;
                        border: none;
                        padding: 10px 20px;
                        font-size: 16px;
                        cursor: pointer;
                        border-radius: 5px;
                        margin: 5px;
                    }
                    button:hover {
                        background: #45a049;
                    }
                </style>
            </head>
            <body>
                <h1>🤖 Person Detection - Pi Camera</h1>
                <div class="info">
                    <p class="status">Detection Active</p>
                    <p>Green boxes = Person detected</p>
                    <p>Confidence threshold: 50%</p>
                </div>
                <br>
                <img src="/stream" />
                <br><br>
                <p>Press Ctrl+C in terminal to stop the server</p>
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
                    
                    # Encode frame as JPEG
                    ret, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ret:
                        continue
                    
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                    time.sleep(0.033)  # ~30 FPS
            except:
                pass
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        pass  # Suppress request logging


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass


def detect_persons(frame):
    """
    Detect persons in the frame using MobileNet SSD
    Returns: frame with bounding boxes, number of persons detected
    """
    global person_count, last_detection_time
    
    (h, w) = frame.shape[:2]
    
    # Prepare blob for the network
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, INPUT_SIZE),
        0.007843,  # Scale factor
        INPUT_SIZE,
        127.5      # Mean subtraction
    )
    
    # Run detection
    net.setInput(blob)
    detections = net.forward()
    
    persons_detected = 0
    
    # Process detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        
        # Filter by confidence and class (person)
        if confidence > CONFIDENCE_THRESHOLD:
            class_id = int(detections[0, 0, i, 1])
            
            if class_id == PERSON_CLASS_ID:
                persons_detected += 1
                
                # Get bounding box coordinates
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                
                # Draw bounding box
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                
                # Draw label
                label = f"Person: {confidence * 100:.1f}%"
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    person_count = persons_detected
    if persons_detected > 0:
        last_detection_time = time.time()
    
    return frame, persons_detected


def capture_and_detect():
    """Capture frames and run person detection"""
    global latest_frame, frame_count, start_time
    
    print("🎥 Starting detection loop...")
    
    while True:
        # Capture frame
        frame = picam2.capture_array()
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        
        # Run person detection
        if detection_enabled:
            frame, persons = detect_persons(frame)
        else:
            persons = 0
        
        # Add info overlay
        info_y = 30
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        info_y += 35
        color = (0, 255, 0) if persons > 0 else (255, 255, 255)
        cv2.putText(frame, f"Persons: {persons}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        info_y += 35
        cv2.putText(frame, f"Frame: {frame_count}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Detection status indicator
        if persons > 0:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 15, (0, 255, 0), -1)
            cv2.putText(frame, "DETECTED", (frame.shape[1] - 150, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        with frame_lock:
            latest_frame = frame
        
        # Small delay to prevent CPU overload on Pi 3B+
        time.sleep(0.01)


def main():
    global picam2, start_time
    
    print("\n" + "="*60)
    print("🤖 PERSON DETECTION - Pi Camera 1.3")
    print("="*60)
    print("Optimized for Raspberry Pi 3B+")
    print("="*60 + "\n")
    
    # Initialize camera
    print("📷 Initializing Pi Camera...")
    picam2 = Picamera2()
    
    # Use lower resolution for better performance on Pi 3B+
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Camera warm-up
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
        print("\nTo find your Pi's IP address, run: hostname -I")
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
        print("\n👋 Goodbye!")


if __name__ == "__main__":
    main()
