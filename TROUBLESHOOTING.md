# Quadbot Troubleshooting Guide

## 🚨 Problem: Servos Making Noise & Program Stuck

### What You're Experiencing:
- ✗ Legs move a bit then stop
- ✗ Servos on one side making buzzing/grinding noise
- ✗ Program gets stuck (infinite loop)
- ✗ Can't Ctrl+C to exit
- ✗ Robot doesn't actually move

### Root Causes:

#### 1. **Infinite Loop in Leg Functions**
The original code has `while` loops that check for exact angle match:
```python
while(channel_cur[0] != angle1 or ...):  # Never exits if can't reach exact angle!
```

**Problem**: If servo can't reach target angle (mechanical limit, offset issue), loop runs forever.

#### 2. **Servo Stalling**
Buzzing noise = servo trying to move but can't because:
- Angle is beyond mechanical limits
- Servo hitting physical obstruction
- Insufficient power causing brownout

#### 3. **Thread Deadlock**
When servo stalls while holding the I2C mutex, other threads can't access servos → deadlock → Ctrl+C doesn't work

#### 4. **Offset Issues**
Some offsets push angles out of valid 0-180° range:
```python
leg2_offset = [0, 10, 0]   # Joint 2: 60 + 10 = 70° ✓
leg3_offset = [0, 0, -10]  # Joint 3: could go negative! ✗
```

---

## ✅ Solutions Provided

### File 1: `quadbot_gaits_robust.py` (RECOMMENDED)

**Key Improvements:**
1. ✅ **Timeout Protection** - Max 200 iterations prevents infinite loops
2. ✅ **Ctrl+C Handler** - Signal handler for clean shutdown
3. ✅ **Angle Clamping** - All angles forced to 0-180° range
4. ✅ **Tolerance-based Exit** - Uses `abs(current - target) > 1` instead of exact match
5. ✅ **Diagnostic Output** - Shows which leg times out and why
6. ✅ **Error Handling** - Try/except blocks prevent crashes
7. ✅ **Shutdown Event** - Global flag to stop all threads cleanly

**Usage:**
```bash
cd /home/ashish/projects/quadbot/quadruped
python3 quadbot_gaits_robust.py
```

Press Ctrl+C anytime to stop safely!

---

### File 2: `servo_diagnostic.py` (DIAGNOSTIC TOOL)

**Purpose:** Test individual servos to identify hardware issues

**Features:**
- Test all 12 servos sequentially
- Test specific servo by channel number
- Check if angle calculations go out of bounds
- Move all servos to neutral position

**Usage:**
```bash
cd /home/ashish/projects/quadbot/quadruped
python3 servo_diagnostic.py
```

**What to Look For:**
- ❌ If any servo fails → hardware issue (bad servo, wiring, power)
- ❌ If servos buzz during test → mechanical obstruction or wrong angle
- ❌ If angle calculations show "OUT OF BOUNDS" → offset configuration issue

---

## 🔍 Diagnostic Steps

### Step 1: Test Hardware
```bash
python3 servo_diagnostic.py
```
Choose option 1 to test all servos. Watch for:
- Servos that don't move
- Servos that buzz/grind
- Servos that move erratically

### Step 2: Check Angle Calculations
In diagnostic tool, choose option 3. This checks if your offsets cause angles to go out of range.

### Step 3: Test Robust Version
```bash
python3 quadbot_gaits_robust.py
```
Watch terminal output for timeout warnings like:
```
⚠️  Leg2 timeout: target=(90,70,110) current=(90,65,110)
```
This tells you which leg/joint is stuck and what angle it's trying to reach.

---

## ⚙️ Configuration Adjustments

### If Movement is Too Slow:
Edit `quadbot_gaits_robust.py`:
```python
move_delay = 0.01   # Reduce from 0.015
step_delay = 0.05   # Reduce from 0.1
servo_step = 5      # Increase from 3 (larger steps)
```

### If Servos Still Stall:
Edit `quadbot_gaits_robust.py`:
```python
move_delay = 0.02   # Increase (slower)
i2c_delay = 0.005   # Increase (more I2C delay)
MAX_ITERATIONS = 300  # Increase timeout
```

### If Specific Leg Has Issues:
Adjust offsets in `quadbot_gaits_robust.py`:
```python
# Example: If leg2 joint2 buzzes at 70°, reduce offset
leg2_offset = [0, 5, 0]  # Changed from [0, 10, 0]
```

---

## 🔧 Hardware Checklist

### Power Supply
- [ ] Servos have **separate 5V power** (NOT from Pi)
- [ ] Power supply is **at least 5A** (preferably 10A for 12 servos)
- [ ] **Common ground** between Pi and servo power
- [ ] **Capacitor** (1000µF or larger) across power rails

### Wiring
- [ ] All servo signal wires connected to correct PCA9685 channels
- [ ] No loose connections
- [ ] Short, thick wires for power (minimize voltage drop)

### Mechanical
- [ ] Servos can move freely (no obstructions)
- [ ] Servo horns properly attached
- [ ] Legs don't hit each other or robot body
- [ ] Servos not fighting against mechanical limits

---

## 📊 Expected Behavior (Robust Version)

### On Startup:
```
==================================================
🤖 QUADBOT GAIT CONTROL - ROBUST VERSION
==================================================
Press Ctrl+C to stop at any time
==================================================

Initializing PCA9685...
Configuring servos...
Servos configured!
🤖 Moving to initial position...
🤖 Moving to stance position...
✅ Robot ready!

🎯 Executing 4 right turns...

Turn 1/4
↻ Turn Right
...
```

### If Issues Occur:
```
⚠️  Leg2 timeout: target=(90,70,110) current=(90,65,110)
```
This means Leg2 couldn't reach 70° on joint 2 (got stuck at 65°).

**Action:** Reduce leg2_offset or check if servo is mechanically blocked.

### On Ctrl+C:
```
🛑 Interrupt received! Shutting down...
🛑 Returning to neutral position...
👋 Goodbye!
```

---

## 🎯 Quick Fix Summary

1. **Use the robust version:**
   ```bash
   python3 quadbot_gaits_robust.py
   ```

2. **If it times out**, check the terminal output to see which leg/joint is stuck

3. **Run diagnostics** to test individual servos:
   ```bash
   python3 servo_diagnostic.py
   ```

4. **Adjust offsets** if angles go out of bounds

5. **Check power supply** - most servo issues are power-related!

---

## 🆘 Still Having Issues?

### Servo Buzzing on One Side:
- **Likely cause:** Offset pushing angle beyond mechanical limit
- **Fix:** Reduce or remove offsets for that leg
- **Test:** Use diagnostic tool to test that specific servo

### Program Still Freezes:
- **Likely cause:** Power supply brownout
- **Fix:** Use larger power supply (10A recommended)
- **Test:** Try with fewer servos moving at once

### Servos Don't Move At All:
- **Likely cause:** I2C communication issue or wrong channel
- **Fix:** Check wiring, verify PCA9685 address (default 0x40)
- **Test:** Run diagnostic tool

---

## 📝 Key Differences: Old vs Robust

| Feature | Old Version | Robust Version |
|---------|-------------|----------------|
| Loop exit | Exact match only | Tolerance-based (±1°) |
| Timeout | None (infinite) | 200 iterations max |
| Ctrl+C | Doesn't work | Clean shutdown |
| Angle validation | None | Clamped to 0-180° |
| Error handling | Crashes | Graceful recovery |
| Diagnostics | None | Timeout warnings |
| Shutdown | Abrupt | Returns to neutral |

---

## 💡 Pro Tips

1. **Always test with diagnostic tool first** before running gaits
2. **Start with neutral position** (all servos at 90°) to verify wiring
3. **Test one leg at a time** to isolate issues
4. **Monitor power supply voltage** - should stay above 4.8V under load
5. **Keep Ctrl+C handy** - robust version will always respond to it!

---

## 🎓 Understanding the Fixes

### Why Tolerance Instead of Exact Match?
```python
# OLD (bad):
while(channel_cur[0] != angle1):  # Might never be exactly equal!

# NEW (good):
while(abs(channel_cur[0] - angle1) > 1):  # Close enough is good enough
```

### Why Timeout Protection?
```python
iterations = 0
while(...):
    if iterations >= MAX_ITERATIONS:
        print("Timeout!")  # Tell user what's wrong
        break  # Exit instead of hanging forever
    iterations += 1
```

### Why Angle Clamping?
```python
def clamp_angle(angle):
    return max(0, min(180, int(angle)))  # Force into valid range

# Prevents: 60 + (-10) = 50 ✓  or  60 + (-70) = -10 ✗
```

---

Good luck! 🚀
