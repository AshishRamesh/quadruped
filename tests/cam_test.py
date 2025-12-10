#!/usr/bin/env python3
"""
Camera Test - Streams Pi Camera over HTTP
Access at http://<raspberry-pi-ip>:8000
"""

from picamera2 import Picamera2
import cv2
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

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head>
                <title>Pi Camera Stream</title>
                <style>
                    body { margin: 0; padding: 20px; background: #1a1a1a; color: white; font-family: Arial; text-align: center; }
                    img { max-width: 90%; border: 2px solid #4CAF50; }
                    h1 { color: #4CAF50; }
                </style>
            </head>
            <body>
                <h1>Pi Camera Live Stream</h1>
                <img src="/stream" />
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
                    ret, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
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

def capture_frames():
    global latest_frame, frame_count, start_time
    
    while True:
        frame = picam2.capture_array()
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        
        # Add FPS and frame info
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        with frame_lock:
            latest_frame = frame

def main():
    global picam2, start_time
    
    print("Starting Pi Camera HTTP stream...")
    
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    
    start_time = time.time()
    
    # Start frame capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    # Start HTTP server
    try:
        server = ThreadedHTTPServer(('0.0.0.0', 8000), StreamingHandler)
        print("\n" + "="*60)
        print("Camera stream running!")
        print("Access at: http://<raspberry-pi-ip>:8000")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0
        picam2.stop()
        print(f"\nCamera closed - {frame_count} frames in {total_time:.2f}s (avg {avg_fps:.2f} FPS)")

if __name__ == "__main__":
    main()

