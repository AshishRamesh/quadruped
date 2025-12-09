#!/usr/bin/env python3
"""
Diagnostic script to test individual servos and identify issues
"""
import time
import sys
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

# Initialize
GPIO.setwarnings(False)
kit = ServoKit(channels=16)

print("\n" + "="*60)
print("🔧 QUADBOT SERVO DIAGNOSTIC TOOL")
print("="*60)

# Configure servos
print("\n📋 Configuring servos...")
for i in range(12):
    kit.servo[i].actuation_range = 180
    time.sleep(0.05)
print("✅ Servos configured!")

def test_servo(channel, name):
    """Test a single servo through its range"""
    print(f"\n🔍 Testing Servo {channel} ({name})")
    print("   Moving to 90° (neutral)...")
    
    try:
        kit.servo[channel].angle = 90
        time.sleep(1)
        
        print("   Moving to 45°...")
        kit.servo[channel].angle = 45
        time.sleep(1)
        
        print("   Moving to 135°...")
        kit.servo[channel].angle = 135
        time.sleep(1)
        
        print("   Returning to 90°...")
        kit.servo[channel].angle = 90
        time.sleep(0.5)
        
        print(f"   ✅ Servo {channel} OK")
        return True
        
    except Exception as e:
        print(f"   ❌ Servo {channel} ERROR: {e}")
        return False

def test_all_servos():
    """Test all 12 servos"""
    servo_names = [
        "Leg1-Joint1", "Leg1-Joint2", "Leg1-Joint3",
        "Leg2-Joint1", "Leg2-Joint2", "Leg2-Joint3",
        "Leg3-Joint1", "Leg3-Joint2", "Leg3-Joint3",
        "Leg4-Joint1", "Leg4-Joint2", "Leg4-Joint3"
    ]
    
    results = []
    for i in range(12):
        result = test_servo(i, servo_names[i])
        results.append(result)
        time.sleep(0.5)
    
    print("\n" + "="*60)
    print("📊 DIAGNOSTIC RESULTS")
    print("="*60)
    
    for i, (name, result) in enumerate(zip(servo_names, results)):
        status = "✅ OK" if result else "❌ FAILED"
        print(f"Servo {i:2d} ({name:15s}): {status}")
    
    failed = sum(1 for r in results if not r)
    if failed == 0:
        print(f"\n✅ All servos working! ({len(results)}/12)")
    else:
        print(f"\n⚠️  {failed} servo(s) failed! ({len(results)-failed}/12 working)")

def test_angles_with_offsets():
    """Test if angle calculations with offsets are valid"""
    print("\n" + "="*60)
    print("🧮 TESTING ANGLE CALCULATIONS WITH OFFSETS")
    print("="*60)
    
    leg1_offset = [0, 0, 0]
    leg2_offset = [0, 10, 0]
    leg3_offset = [0, 0, -10]
    leg4_offset = [0, 0, -10]
    
    test_angles = [
        ("front_parallel, footdown, pincer_down", 90, 60, 120),
        ("back_parallel, footdown, pincer_down", 90, 60, 120),
        ("back_lateral, footdown, pincer_down", 140, 60, 120),
        ("front_lateral, footdown, pincer_down", 40, 60, 120),
        ("footup, pincer_up", 0, 0, 130),
    ]
    
    legs = [
        ("Leg1", leg1_offset),
        ("Leg2", leg2_offset),
        ("Leg3", leg3_offset),
        ("Leg4", leg4_offset),
    ]
    
    print("\nChecking if any angles go out of bounds (0-180):\n")
    
    issues_found = False
    for leg_name, offset in legs:
        print(f"{leg_name} (offset={offset}):")
        for desc, a1, a2, a3 in test_angles:
            final1 = a1 + offset[0]
            final2 = a2 + offset[1]
            final3 = a3 + offset[2]
            
            out_of_bounds = []
            if final1 < 0 or final1 > 180:
                out_of_bounds.append(f"J1={final1}")
            if final2 < 0 or final2 > 180:
                out_of_bounds.append(f"J2={final2}")
            if final3 < 0 or final3 > 180:
                out_of_bounds.append(f"J3={final3}")
            
            if out_of_bounds:
                print(f"  ⚠️  {desc}: OUT OF BOUNDS! {', '.join(out_of_bounds)}")
                issues_found = True
    
    if not issues_found:
        print("  ✅ All angles within valid range!")
    
    print()

def main():
    """Main menu"""
    while True:
        print("\n" + "="*60)
        print("MENU:")
        print("  1. Test all servos sequentially")
        print("  2. Test specific servo")
        print("  3. Check angle calculations")
        print("  4. Move all servos to neutral (90°)")
        print("  5. Exit")
        print("="*60)
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == '1':
            test_all_servos()
        elif choice == '2':
            try:
                channel = int(input("Enter servo channel (0-11): "))
                if 0 <= channel <= 11:
                    test_servo(channel, f"Servo-{channel}")
                else:
                    print("❌ Invalid channel! Must be 0-11")
            except ValueError:
                print("❌ Invalid input!")
        elif choice == '3':
            test_angles_with_offsets()
        elif choice == '4':
            print("\n🔄 Moving all servos to 90°...")
            for i in range(12):
                kit.servo[i].angle = 90
                time.sleep(0.05)
            print("✅ Done!")
        elif choice == '5':
            print("\n👋 Exiting...")
            # Return all to neutral
            for i in range(12):
                kit.servo[i].angle = 90
                time.sleep(0.05)
            GPIO.cleanup()
            sys.exit(0)
        else:
            print("❌ Invalid choice!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted!")
        for i in range(12):
            kit.servo[i].angle = 90
        GPIO.cleanup()
        sys.exit(0)
