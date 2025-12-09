#!/usr/bin/env python3
"""
Lightweight Person Detection using MediaPipe Pose
Optimized for Raspberry Pi 3B+ - No model download required!
MediaPipe is faster and lighter than MobileNet SSD on Pi 3B+
"""

from picamera2 import Picamera2
import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("⚠️  MediaPipe not installed!")
    print("Install with: pip3 install mediapipe")
    exit(1)

# Global variables
picam2 = None
frame_count = 0
start_time = None
latest_frame = None
frame_lock = threading.Lock()
person_detected = False
detection_confidence = 0.0

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Create pose detector with optimized settings for Pi 3B+
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,  # 0=Lite (fastest), 1=Full, 2=Heavy
    smooth_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

print("✅ MediaPipe Pose initialized!")


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head>
                <title>Person Detection - MediaPipe</title>
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
                        border: 3px solid #FF6B6B; 
                        border-radius: 8px;
                    }
                    h1 { 
                        color: #FF6B6B; 
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
                        color: #FF6B6B;
                        font-size: 18px;
                        font-weight: bold;
                    }
                    .detected {
                        color: #4CAF50;
                    }
                </style>
            </head>
            <body>
                <h1>🤖 Person Detection - MediaPipe Pose</h1>
                <div class="info">
                    <p class="status">Lightweight Detection Active</p>
                    <p>Shows skeleton overlay when person detected</p>
                    <p>Optimized for Pi 3B+</p>
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


def detect_person_pose(frame):
    """
    Detect person using MediaPipe Pose
    Returns: frame with pose overlay, detection status, confidence
    """
    # Convert BGR to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the frame
    results = pose.process(rgb_frame)
    
    detected = False
    confidence = 0.0
    
    if results.pose_landmarks:
        detected = True
        
        # Draw pose landmarks on the frame
        mp_drawing.draw_landmarks(
            frame,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style()
        )
        
        # Calculate average visibility as confidence
        visibilities = [lm.visibility for lm in results.pose_landmarks.landmark]
        confidence = sum(visibilities) / len(visibilities)
        
        # Draw bounding box around person
        h, w, _ = frame.shape
        landmarks = results.pose_landmarks.landmark
        
        # Get min/max coordinates
        x_coords = [lm.x * w for lm in landmarks]
        y_coords = [lm.y * h for lm in landmarks]
        
        x_min, x_max = int(min(x_coords)), int(max(x_coords))
        y_min, y_max = int(min(y_coords)), int(max(y_coords))
        
        # Add padding
        padding = 20
        x_min = max(0, x_min - padding)
        y_min = max(0, y_min - padding)
        x_max = min(w, x_max + padding)
        y_max = min(h, y_max + padding)
        
        # Draw bounding box
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        
        # Draw label
        label = f"Person: {confidence * 100:.1f}%"
        cv2.putText(frame, label, (x_min, y_min - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame, detected, confidence


def capture_and_detect():
    """Capture frames and run person detection"""
    global latest_frame, frame_count, start_time, person_detected, detection_confidence
    
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
        frame, detected, confidence = detect_person_pose(frame)
        person_detected = detected
        detection_confidence = confidence
        
        # Add info overlay
        info_y = 30
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        info_y += 35
        status_text = "PERSON DETECTED" if detected else "No person"
        color = (0, 255, 0) if detected else (255, 255, 255)
        cv2.putText(frame, status_text, (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        if detected:
            info_y += 35
            cv2.putText(frame, f"Confidence: {confidence * 100:.1f}%", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        info_y += 35
        cv2.putText(frame, f"Frame: {frame_count}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Detection status indicator
        if detected:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 15, (0, 255, 0), -1)
        else:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 15, (128, 128, 128), 2)
        
        with frame_lock:
            latest_frame = frame
        
        # Small delay to prevent CPU overload
        time.sleep(0.01)


def main():
    global picam2, start_time
    
    print("\n" + "="*60)
    print("🤖 PERSON DETECTION - MediaPipe Pose")
    print("="*60)
    print("Lightweight & Optimized for Raspberry Pi 3B+")
    print("="*60 + "\n")
    
    # Initialize camera
    print("📷 Initializing Pi Camera...")
    picam2 = Picamera2()
    
    # Use 640x480 for good balance of speed and accuracy
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
        pose.close()
        picam2.stop()
        print(f"\n📊 Statistics:")
        print(f"   Total frames: {frame_count}")
        print(f"   Runtime: {total_time:.2f}s")
        print(f"   Average FPS: {avg_fps:.2f}")
        print("\n👋 Goodbye!")


if __name__ == "__main__":
    main()
