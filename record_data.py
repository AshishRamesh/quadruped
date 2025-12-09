#!/usr/bin/env python3
"""
Record data from Pi Camera
Captures images continuously and saves them with sequential numbering
Press ESC to exit
"""

from picamera2 import Picamera2
import cv2
import time
import numpy as np

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(1)  # Camera warm-up

imcount = 0
saveimg = 1

print("Recording started. Press ESC to stop.")
print(f"Saving images with prefix '00'")

try:
    while True:
        # Capture frame from Pi Camera
        img = picam2.capture_array()
        
        # Display the image
        cv2.imshow('image', img)
        
        if saveimg == 1:
            imagename = "00" + str(imcount) + ".jpg"
            # Convert RGB to BGR for proper color saving
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(imagename, img_bgr)
            print(f"Saved: {imagename}")
            time.sleep(0.1)
            imcount = imcount + 1
        
        k = cv2.waitKey(30) & 0xff
        if k == 27:  # ESC key
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print(f"Recording stopped. Total images saved: {imcount}")
