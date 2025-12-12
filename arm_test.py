#!/usr/bin/env python3
"""
ARM Test Script - Tests arming capability without flight
REMOVE PROPELLERS BEFORE RUNNING!
Optimized for Raspberry Pi 3
"""

from dronekit import connect, VehicleMode
import time
import sys
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="ARM test script with optional flags to force or set parameters")
    parser.add_argument('--force', action='store_true', help='Skip parameter warning prompt and continue')
    parser.add_argument('--set-params', dest='set_params', action='store_true', help='Attempt to write ARMING_CHECK, FS_THR_ENABLE, FS_GCS_ENABLE = 0 before testing')
    parser.add_argument('--connection', type=str, default='/dev/serial0', help='Connection string (default: /dev/serial0)')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate (default: 57600)')
    return parser.parse_args()

# ============================================
# FUNCTIONS
# ============================================

def connect_vehicle(connection_string, baud_rate):
    """Connect to vehicle"""
    print("\n" + "="*50)
    print("CONNECTING TO AUTUMN - 2")
    print("="*50)
    print("Port: %s" % connection_string)
    print("Baud: %d" % baud_rate)
    
    try:
        vehicle = connect(connection_string, baud=baud_rate, wait_ready=True, timeout=60)
        print("✓ Connected successfully!")
        return vehicle
    except Exception as e:
        print("✗ Connection failed: %s" % str(e))
        sys.exit(1)

def display_parameters(vehicle):
    """Display critical parameters"""
    print("\n" + "="*50)
    print("CRITICAL PARAMETERS")
    print("="*50)
    
    params = {
        'ARMING_CHECK': vehicle.parameters.get('ARMING_CHECK'),
        'FS_THR_ENABLE': vehicle.parameters.get('FS_THR_ENABLE'),
        'FS_GCS_ENABLE': vehicle.parameters.get('FS_GCS_ENABLE')
    }
    
    for param, value in params.items():
        status = "✓ OK" if value == 0 else "✗ SHOULD BE 0"
        print("%s = %s  %s" % (param.ljust(20), str(value).ljust(5), status))
    
    return all(v == 0 for v in params.values())

def display_vehicle_state(vehicle):
    """Display current vehicle state"""
    print("\n" + "="*50)
    print("VEHICLE STATE")
    print("="*50)
    
    print("Autopilot Version: %s" % vehicle.version)
    print("System Status: %s" % vehicle.system_status.state)
    print("Mode: %s" % vehicle.mode.name)
    print("Armed: %s" % vehicle.armed)
    print("Armable: %s" % vehicle.is_armable)
    
    print("\nGPS Status:")
    print("  Fix Type: %s" % vehicle.gps_0.fix_type)
    print("  Satellites: %s" % vehicle.gps_0.satellites_visible)
    print("  Location: %s" % vehicle.location.global_frame)
    
    print("\nBattery:")
    print("  Voltage: %.2f V" % vehicle.battery.voltage)
    print("  Current: %.2f A" % (vehicle.battery.current or 0))
    print("  Level: %s%%" % (vehicle.battery.level or "N/A"))
    
    print("\nHome Location:")
    print("  %s" % vehicle.home_location)

def check_prerequisites(vehicle):
    """Check if vehicle is ready to arm"""
    print("\n" + "="*50)
    print("PRE-ARM CHECKS")
    print("="*50)
    
    checks_passed = True
    
    # Check GPS
    if vehicle.gps_0.fix_type < 2:
        print("✗ GPS: No fix (fix_type: %d)" % vehicle.gps_0.fix_type)
        print("  Need: fix_type >= 2 (2D fix minimum)")
        print("  Satellites: %d (need 6+)" % vehicle.gps_0.satellites_visible)
        checks_passed = False
    else:
        print("✓ GPS: Fix type %d with %d satellites" % 
              (vehicle.gps_0.fix_type, vehicle.gps_0.satellites_visible))
    
    # Check battery
    if vehicle.battery.voltage < 10.0:
        print("✗ Battery: %.2fV (too low!)" % vehicle.battery.voltage)
        checks_passed = False
    else:
        print("✓ Battery: %.2fV" % vehicle.battery.voltage)
    
    # Check mode
    print("✓ Mode: %s" % vehicle.mode.name)
    
    # Check armable status
    if not vehicle.is_armable:
        print("✗ Vehicle reports NOT ARMABLE")
        print("  Waiting for armable status...")
        checks_passed = False
    else:
        print("✓ Vehicle is ARMABLE")
    
    return checks_passed

def wait_for_armable(vehicle, timeout=30):
    """Wait for vehicle to become armable"""
    print("\n" + "="*50)
    print("WAITING FOR ARMABLE STATUS")
    print("="*50)
    
    start_time = time.time()
    
    while not vehicle.is_armable:
        elapsed = time.time() - start_time
        
        if elapsed > timeout:
            print("\n✗ Timeout after %d seconds" % timeout)
            print("\nPossible issues:")
            print("  1. GPS doesn't have enough satellites")
            print("  2. Parameters not set correctly")
            print("  3. Hardware issue with sensors")
            return False
        
        print("  Waiting... (%.0fs) | GPS: %d sats, fix: %d" % 
              (elapsed, vehicle.gps_0.satellites_visible, vehicle.gps_0.fix_type))
        time.sleep(1)
    
    print("\n✓ Vehicle is now ARMABLE!")
    return True

def test_arm(vehicle):
    """Test arming the vehicle"""
    print("\n" + "="*50)
    print("ARMING TEST")
    print("="*50)
    
    # Set GUIDED mode
    print("\nStep 1: Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    
    if vehicle.mode.name != "GUIDED":
        print("✗ Failed to set GUIDED mode!")
        print("  Current mode: %s" % vehicle.mode.name)
        return False
    
    print("✓ Mode set to GUIDED")
    
    # Arm motors
    print("\nStep 2: Arming motors...")
    print("⚠️  Motors will spin! Ensure propellers are REMOVED!")
    time.sleep(2)
    
    vehicle.armed = True
    
    # Wait for arming
    timeout = 10
    start_time = time.time()
    
    while not vehicle.armed:
        if time.time() - start_time > timeout:
            print("✗ Arming failed after %d seconds" % timeout)
            print("\nCheck Mission Planner console for pre-arm error messages")
            return False
        
        print("  Waiting for arm confirmation...")
        time.sleep(0.5)
    
    print("\n" + "="*50)
    print("✓✓✓ ARMED SUCCESSFULLY! ✓✓✓")
    print("="*50)
    print("Motors should be spinning now (if propellers were on)")
    
    return True

def test_disarm(vehicle):
    """Test disarming the vehicle"""
    print("\n" + "="*50)
    print("DISARMING TEST")
    print("="*50)
    
    print("Disarming in 3 seconds...")
    time.sleep(3)
    
    vehicle.armed = False
    
    # Wait for disarm confirmation
    timeout = 5
    start_time = time.time()
    
    while vehicle.armed:
        if time.time() - start_time > timeout:
            print("✗ Disarm timeout!")
            return False
        
        print("  Waiting for disarm confirmation...")
        time.sleep(0.5)
    
    print("✓ DISARMED successfully!")
    return True

# ============================================
# MAIN PROGRAM
# ============================================

def main():
    vehicle = None
    args = parse_args()
    
    try:
        print("\n" + "="*50)
        print("ARM TEST SCRIPT - NO FLIGHT")
        print("Raspberry Pi 3 Version")
        print("="*50)
        print("\n⚠️  WARNING: This will ARM the motors!")
        print("⚠️  REMOVE ALL PROPELLERS before continuing!")
        print("⚠️  Motors will spin when armed!\n")
        
        response = input("Have you removed propellers? Type 'YES': ")
        if response != "YES":
            print("\nTest cancelled. Remove propellers first!")
            sys.exit(0)
        
        # Connect
        vehicle = connect_vehicle(args.connection, args.baud)
        
        # Display parameters
        params_ok = display_parameters(vehicle)
        if not params_ok:
            print("\n⚠️  WARNING: Parameters not set correctly!")
            print("Detected parameters should be 0 for this test:")
            print("  ARMING_CHECK = 0")
            print("  FS_THR_ENABLE = 0")
            print("  FS_GCS_ENABLE = 0")

            if args.set_params:
                print("\nAttempting to write parameters to vehicle (ARMING_CHECK, FS_THR_ENABLE, FS_GCS_ENABLE = 0)")
                try:
                    vehicle.parameters['ARMING_CHECK'] = 0
                    vehicle.parameters['FS_THR_ENABLE'] = 0
                    vehicle.parameters['FS_GCS_ENABLE'] = 0
                    # small pause to allow autopilot to apply
                    time.sleep(1)
                    print("Parameters written; re-reading:")
                    params_ok = display_parameters(vehicle)
                except Exception as e:
                    print("✗ Failed to write parameters: %s" % str(e))
                    print("Proceeding to prompt user")

            if not params_ok:
                if args.force:
                    print("\n--force supplied: continuing despite parameters not being 0 (USE WITH CAUTION)")
                else:
                    response = input("\nContinue anyway? (yes/no): ")
                    if response.lower() != "yes":
                        vehicle.close()
                        sys.exit(0)
        
        # Display state
        display_vehicle_state(vehicle)
        
        # Check prerequisites
        checks_ok = check_prerequisites(vehicle)
        
        # Wait for armable if needed
        if not checks_ok or not vehicle.is_armable:
            if not wait_for_armable(vehicle):
                print("\n✗ Vehicle not ready to arm")
                vehicle.close()
                sys.exit(1)
        
        # User confirmation
        print("\n" + "="*50)
        response = input("Ready to ARM? Type 'ARM': ")
        if response != "ARM":
            print("Test cancelled.")
            vehicle.close()
            sys.exit(0)
        
        # Test arm
        arm_success = test_arm(vehicle)
        
        if arm_success:
            # Keep armed for 5 seconds
            print("\nKeeping armed for 5 seconds...")
            for i in range(5, 0, -1):
                print("  %d..." % i)
                time.sleep(1)
            
            # Test disarm
            test_disarm(vehicle)
            
            print("\n" + "="*50)
            print("✓✓✓ ARM TEST COMPLETED SUCCESSFULLY! ✓✓✓")
            print("="*50)
            print("\nNext steps:")
            print("  1. If motors didn't spin, check ESC connections")
            print("  2. If test passed, you're ready for takeoff test")
            print("  3. For takeoff test, use tether or low altitude first")
        else:
            print("\n✗✗✗ ARM TEST FAILED ✗✗✗")
            print("Check error messages above")
        
    except KeyboardInterrupt:
        print("\n\nTest aborted by user!")
        if vehicle and vehicle.armed:
            print("Disarming...")
            vehicle.armed = False
            time.sleep(2)
    
    except Exception as e:
        print("\n✗ Error occurred: %s" % str(e))
        import traceback
        traceback.print_exc()
        
        if vehicle and vehicle.armed:
            print("Emergency disarm...")
            vehicle.armed = False
            time.sleep(2)
    
    finally:
        if vehicle:
            print("\nClosing connection...")
            vehicle.close()
            print("Connection closed.")

if __name__ == "__main__":
    main()
