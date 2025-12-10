# Quadbot Pi Crash Troubleshooting Guide

## 🚨 Problem: Pi Crashes After Some Time

### Common Causes:

1. **Power Supply Issues** (MOST COMMON)
   - 12 servos draw significant current
   - Voltage drops cause Pi to brownout/crash
   - SSH disconnects when Pi loses power

2. **Overheating**
   - Pi 3B+ gets hot under load
   - Thermal throttling → crash

3. **I2C Bus Errors**
   - Too many I2C commands
   - Bus lockup

---

## ✅ Solutions Applied in `quadbot_gaits_robust.py`

### Version History:
- **v1**: Used wrong library (ServoKit) → immediate crash
- **v2**: Used CircuitPython PCA9685 → worked better but still crashed
- **v3 (CURRENT)**: Ultra-conservative mode with:

### Changes Made:
1. **Slower Movement**
   - `move_delay`: 0.0005 → 0.02 (40x slower!)
   - `step_delay`: 0.001 → 0.15 (150x slower!)
   - `servo_step`: 2 → 1 degree (smoother)

2. **I2C Protection**
   - Retry logic (3 attempts)
   - Longer I2C delays (5ms)
   - Better error handling

3. **Power Management**
   - Longer delays between leg movements (0.5s)
   - Only one leg moves at a time during init
   - Extra settling time (1-1.5s)

4. **Temperature Monitoring**
   - Shows CPU temp before/during/after
   - Auto-pauses if temp > 75°C
   - Warns if temp > 70°C

---

## 🔧 Hardware Fixes (CRITICAL!)

### 1. Power Supply (MOST IMPORTANT!)

**Problem**: Pi and servos sharing power → voltage drops → crash

**Solution**:
```
┌─────────────┐
│  5V 10A PSU │
└──┬──────┬───┘
   │      │
   │      └─→ PCA9685 V+ (servo power)
   │
   └─→ Pi 5V (Pi power)
   
IMPORTANT: Connect GND together!
```

**Requirements**:
- **Separate 5V power for servos** (NOT from Pi!)
- **At least 10A power supply** (12 servos × 0.5-1A each)
- **Common ground** between Pi and servo power
- **Thick wires** (18-20 AWG) for power

**Quick Test**:
```bash
# Monitor voltage while running
vcgencmd get_throttled
# If returns 0x50000 or 0x50005 → under-voltage!
```

### 2. Add Capacitors

**Problem**: Servo movement causes voltage spikes

**Solution**:
- Add **1000-2200µF capacitor** across PCA9685 V+ and GND
- Smooths voltage spikes
- Prevents brownouts

### 3. Cooling

**Problem**: Pi overheats → thermal throttling → crash

**Solution**:
- Add **heatsinks** to Pi CPU
- Use **active cooling** (small fan)
- Ensure **good airflow**

**Check Temperature**:
```bash
vcgencmd measure_temp
# Should stay below 70°C
```

---

## 🧪 Diagnostic Steps

### Step 1: Check Power
```bash
# Run this WHILE robot is moving
vcgencmd get_throttled

# Decode result:
# 0x0 = OK
# 0x50000 = Under-voltage occurred
# 0x50005 = Currently under-voltage
```

### Step 2: Monitor Temperature
```bash
# Watch temperature in real-time
watch -n 1 vcgencmd measure_temp

# Or use the script (it shows temp automatically)
python3 quadbot_gaits_robust.py
```

### Step 3: Check I2C
```bash
# Scan I2C bus
i2cdetect -y 1

# Should show PCA9685 at address 0x40
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

### Step 4: Test with Fewer Servos
Temporarily modify code to test with just 1-2 servos:
```python
def begin():
    print("Testing with leg1 only...")
    leg1(89, 89, 89)
    time.sleep(1)
    leg1(front_parallel, footdown, pincer_down)
    # Comment out leg2, leg3, leg4
```

If this works → power issue!

---

## ⚙️ Configuration Tuning

### If Still Crashing:

#### Make it EVEN SLOWER:
Edit `quadbot_gaits_robust.py`:
```python
move_delay = 0.03   # Even slower (30ms)
step_delay = 0.2    # Even more delay (200ms)
servo_step = 1      # Already at minimum
```

#### Reduce Simultaneous Movement:
In `turn_right()` and other functions, avoid threading:
```python
# Instead of:
t2 = Thread(target=leg2, args=(...))
t3 = Thread(target=leg3, args=(...))
t2.start()
t3.start()

# Do sequentially:
leg2(...)
time.sleep(0.5)
leg3(...)
time.sleep(0.5)
```

---

## 📊 Expected Behavior (Ultra-Conservative Mode)

### Startup:
```
🤖 QUADBOT GAIT CONTROL - CircuitPython PCA9685
ULTRA-CONSERVATIVE MODE for Pi 3B+ stability
🌡️  CPU Temperature: 45.2°C
✅ GPIO pins configured!
✅ PCA9685 initialized!
🤖 Moving to initial position...
🤖 Moving to stance position...
✅ Robot ready!
🌡️  CPU Temperature after init: 48.5°C
```

### During Movement:
```
Turn 1/4
↻ Turn Right
⚠️  CPU Warm: 52.3°C

Turn 2/4
↻ Turn Right
```

### If Overheating:
```
Turn 3/4
↻ Turn Right
⚠️  CPU HOT: 76.2°C - Adding cooling delay...
(waits 2 seconds)
```

---

## 🆘 Still Crashing?

### Check These:

1. **Power Supply Voltage**
   ```bash
   # Should be 5.0-5.2V under load
   vcgencmd measure_volts
   ```

2. **Power Supply Current Rating**
   - Need at least 10A for 12 servos
   - Many "5A" supplies can't deliver full current

3. **Wiring**
   - Check for loose connections
   - Ensure thick power wires
   - Verify common ground

4. **Servo Quality**
   - Cheap servos draw more current
   - Some servos stall and draw excessive current

5. **Mechanical Issues**
   - Servos fighting against mechanical limits
   - Legs hitting each other
   - Binding in joints

---

## 💡 Quick Fixes to Try

### 1. Reduce Number of Active Servos
Test with 6 servos (2 legs) instead of 12:
```python
def begin():
    leg1(89, 89, 89)
    leg2(89, 89, 89)
    # Comment out leg3, leg4
```

### 2. Use External Power ONLY for Servos
Don't power Pi from same supply as servos.

### 3. Add Delays Everywhere
Can't have too many delays when debugging power issues!

### 4. Monitor System Logs
```bash
# Check for kernel panics or power issues
dmesg | tail -50
journalctl -xe
```

---

## 📝 Checklist Before Running

- [ ] Servos have separate 5V power (NOT from Pi)
- [ ] Power supply is 10A or higher
- [ ] Common ground connected
- [ ] Capacitor across servo power rails
- [ ] Pi has heatsink/cooling
- [ ] All wiring is secure
- [ ] CPU temperature < 60°C before start
- [ ] Using `quadbot_gaits_robust.py` (latest version)

---

## 🎯 Success Criteria

If working correctly, you should see:
- ✅ Robot completes all 4 turns without crash
- ✅ SSH stays connected
- ✅ CPU temp stays below 70°C
- ✅ No voltage warnings
- ✅ Smooth servo movement

---

## 📞 If Nothing Works

The issue is almost certainly **power supply**. Try:

1. **Test with USB power bank** (5V 3A) for Pi only
2. **Use car battery** (12V → 5V regulator) for servos
3. **Reduce to 6 servos** (2 legs only)
4. **Check servo current draw** with multimeter

Remember: **12 servos can draw 6-12A under load!**

---

Good luck! 🚀
