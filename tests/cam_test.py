#!/usr/bin/env python3
"""
Simple Camera Test - Shows Pi Camera window for 5 seconds
"""

from picamera2 import Picamera2
import cv2
import time

def main():
    print("Starting Pi Camera test...")
    
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    
    print("Camera running for 5 seconds...")
    start_time = time.time()
    frame_count = 0
    
    try:
        while time.time() - start_time < 5:
            frame = picam2.capture_array()
            # Picamera2 gives RGB, OpenCV displays BGR - no conversion needed for display
            
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Add FPS and frame info to the image
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Frame: {frame_count}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Time: {elapsed:.1f}s / 5s", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow("Pi Camera Test", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0
        
        picam2.stop()
        cv2.destroyAllWindows()
        
        print(f"Camera closed - {frame_count} frames in {total_time:.2f}s (avg {avg_fps:.2f} FPS)")

if __name__ == "__main__":
    main()

