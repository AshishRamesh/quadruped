# Person Detection - Quick Reference

## 🚀 Quick Start

### Option 1: Automated Setup (Recommended)
```bash
cd /home/ashish/projects/quadbot/quadruped
chmod +x setup_person_detection.sh
./setup_person_detection.sh
```

### Option 2: Manual Setup
```bash
# Install dependencies
pip3 install --user mediapipe opencv-python

# Run detection
python3 person_detection_mediapipe.py

# Access at: http://<pi-ip>:8000
```

---

## 📁 Files Created

1. **`person_detection_mediapipe.py`** ⭐ **RECOMMENDED**
   - Lightweight MediaPipe Pose detection
   - 5-10 FPS on Pi 3B+
   - No model download needed
   - Shows skeleton overlay

2. **`person_detection.py`**
   - MobileNet SSD detection
   - 2-5 FPS on Pi 3B+
   - Requires model download
   - Shows bounding boxes

3. **`setup_person_detection.sh`**
   - Automated installation script
   - Installs all dependencies
   - Downloads models (optional)

4. **`PERSON_DETECTION_SETUP.md`**
   - Complete setup guide
   - Troubleshooting tips
   - Performance optimization

---

## 🎯 Which One to Use?

### Use MediaPipe (`person_detection_mediapipe.py`) if:
- ✅ You want better FPS on Pi 3B+
- ✅ You want easier setup (no model download)
- ✅ You want pose/skeleton visualization
- ✅ You only need to detect people (not other objects)

### Use MobileNet SSD (`person_detection.py`) if:
- ✅ You need higher accuracy
- ✅ You want to detect multiple people
- ✅ You want to detect other objects (cars, dogs, etc.)
- ✅ You don't mind slower FPS

---

## 📊 Performance on Pi 3B+

| Script | FPS | CPU | Setup Difficulty |
|--------|-----|-----|------------------|
| MediaPipe | 5-10 | ~60% | Easy ⭐ |
| MobileNet SSD | 2-5 | ~80% | Medium |

---

## 🔧 Basic Usage

### Run MediaPipe Version:
```bash
python3 person_detection_mediapipe.py
```

### Run MobileNet Version:
```bash
# First download models (one-time setup)
mkdir -p models && cd models
wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.prototxt
wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.caffemodel
cd ..

# Then run
python3 person_detection.py
```

### View Stream:
1. Find your Pi's IP: `hostname -I`
2. Open browser: `http://<pi-ip>:8000`
3. Press Ctrl+C to stop

---

## 💡 Tips

### Improve FPS:
- Use lower resolution (320x240)
- Reduce detection confidence threshold
- Close other applications
- Ensure good cooling

### Better Detection:
- Ensure good lighting
- Stand 1-5 meters from camera
- Face camera directly
- Less cluttered background

---

## 🆘 Troubleshooting

### Camera not working:
```bash
libcamera-hello  # Test camera
sudo raspi-config  # Enable camera interface
```

### Low FPS:
- Reduce resolution in code
- Use MediaPipe version
- Check CPU temperature: `vcgencmd measure_temp`

### Can't access stream:
- Check firewall: `sudo ufw allow 8000`
- Verify IP: `hostname -I`
- Try locally: `http://localhost:8000`

---

## 🔗 Integration Example

Integrate with quadbot for person following:

```python
from person_detection_mediapipe import detect_person_pose
from quadbot_gaits_robust import forward, turn_right

frame = picam2.capture_array()
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
_, detected, confidence = detect_person_pose(frame)

if detected and confidence > 0.7:
    forward()  # Follow person
else:
    turn_right()  # Search for person
```

---

For complete documentation, see **`PERSON_DETECTION_SETUP.md`**
