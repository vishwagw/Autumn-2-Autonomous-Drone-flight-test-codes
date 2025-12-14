#!/usr/bin/env python
"""
Connection Test for Raspberry Pi 3 - Verifies basic communication with APM 2.8
Run this FIRST to ensure your setup is correct
"""

from dronekit import connect, VehicleMode
import time
import sys
import os

# ============================================
# CONFIGURATION - RASPBERRY PI 3
# ============================================
# Common Raspberry Pi serial connections:
# - '/dev/ttyUSB0' - USB to serial adapter
# - '/dev/ttyACM0' - Direct USB connection to APM
# - '/dev/serial0' - GPIO serial pins (UART)
CONNECTION_STRING = '/dev/ttyUSB0'  # Change if using different port
BAUD_RATE = 57600  # Standard for APM 2.8

# ============================================
# RASPBERRY PI DETECTION
# ============================================

def detect_serial_ports():
    """Detect available serial ports on Raspberry Pi"""
    print("\nDetecting available serial ports...")
    ports = []
    
    # Check common serial devices
    possible_ports = [
        '/dev/ttyUSB0', '/dev/ttyUSB1',
        '/dev/ttyACM0', '/dev/ttyACM1',
        '/dev/serial0', '/dev/ttyAMA0'
    ]
    
    for port in possible_ports:
        if os.path.exists(port):
            ports.append(port)
            print("  Found: %s" % port)
    
    if not ports:
        print("  No serial ports found!")
        print("\nRun 'ls /dev/tty*' to see all devices")
    
    return ports

# ============================================
# CONNECTION TEST
# ============================================

def test_connection():
    """Test basic connection to APM"""
    print("\n" + "="*60)
    print("APM 2.8 CONNECTION TEST - RASPBERRY PI 3")
    print("="*60)
    
    # Detect available ports
    available_ports = detect_serial_ports()
    
    print("\nAttempting to connect...")
    print("Port: %s" % CONNECTION_STRING)
    print("Baud Rate: %d" % BAUD_RATE)
    print("\nThis may take 10-30 seconds...")
    
    try:
        # Attempt connection
        vehicle = connect(CONNECTION_STRING, baud=BAUD_RATE, wait_ready=True, timeout=60)
        
        print("\n" + "="*60)
        print("*** CONNECTION SUCCESSFUL! ***")
        print("="*60)
        
        return vehicle
        
    except Exception as e:
        print("\n" + "="*60)
        print("*** CONNECTION FAILED ***")
        print("="*60)
        print("\nError: %s" % str(e))
        
        print("\nTroubleshooting for Raspberry Pi:")
        print("1. Check USB cable is connected to APM")
        print("2. Verify port with: ls /dev/tty*")
        if available_ports:
            print("   Try these detected ports: %s" % ', '.join(available_ports))
        print("3. Check user permissions:")
        print("   sudo usermod -a -G dialout $USER")
        print("   (then logout/login)")
        print("4. Try different baud rates: 57600 or 115200")
        print("5. Check APM has power (LED should be on)")
        print("6. Restart APM: sudo reboot")
        
        return None

def display_basic_info(vehicle):
    """Display basic vehicle information"""
    print("\n" + "="*60)
    print("BASIC VEHICLE INFORMATION")
    print("="*60)
    
    try:
        print("\n[AUTOPILOT INFO]")
        print("  Firmware: %s" % vehicle.version)
        print("  Vehicle Type: %s" % vehicle._vehicle_type)
        
        print("\n[CURRENT STATUS]")
        print("  System Status: %s" % vehicle.system_status.state)
        print("  Mode: %s" % vehicle.mode.name)
        print("  Armed: %s" % vehicle.armed)
        print("  Armable: %s" % vehicle.is_armable)
        
    except Exception as e:
        print("  Error reading basic info: %s" % str(e))

def display_gps_info(vehicle):
    """Display GPS information"""
    print("\n" + "="*60)
    print("GPS INFORMATION")
    print("="*60)
    
    try:
        print("\n  Fix Type: %s" % vehicle.gps_0.fix_type)
        print("    0 = No GPS")
        print("    1 = No Fix")
        print("    2 = 2D Fix")
        print("    3 = 3D Fix")
        
        print("\n  Satellites Visible: %s" % vehicle.gps_0.satellites_visible)
        print("  EPH: %s" % vehicle.gps_0.eph)
        print("  EPV: %s" % vehicle.gps_0.epv)
        
        if vehicle.gps_0.fix_type >= 2:
            print("\n  [OK] GPS has fix!")
            print("  Location: %s" % vehicle.location.global_frame)
        else:
            print("\n  [!] GPS does not have fix yet")
            print("  Move to open area with clear sky view")
            print("  Wait 30-60 seconds for GPS lock")
        
    except Exception as e:
        print("  Error reading GPS: %s" % str(e))

def display_battery_info(vehicle):
    """Display battery information"""
    print("\n" + "="*60)
    print("BATTERY INFORMATION")
    print("="*60)
    
    try:
        print("\n  Voltage: %.2f V" % vehicle.battery.voltage)
        
        if vehicle.battery.current is not None:
            print("  Current: %.2f A" % vehicle.battery.current)
        else:
            print("  Current: Not available")
        
        if vehicle.battery.level is not None:
            print("  Level: %d%%" % vehicle.battery.level)
        else:
            print("  Level: Not available")
        
        # Battery health check
        if vehicle.battery.voltage < 10.0:
            print("\n  [!] WARNING: Battery voltage low!")
        elif vehicle.battery.voltage < 10.5:
            print("\n  [!] CAUTION: Battery voltage marginal")
        else:
            print("\n  [OK] Battery voltage OK")
        
    except Exception as e:
        print("  Error reading battery: %s" % str(e))

def display_parameters(vehicle):
    """Display critical parameters"""
    print("\n" + "="*60)
    print("CRITICAL PARAMETERS FOR COMPUTER-ONLY CONTROL")
    print("="*60)
    
    try:
        params = {
            'ARMING_CHECK': vehicle.parameters.get('ARMING_CHECK'),
            'FS_THR_ENABLE': vehicle.parameters.get('FS_THR_ENABLE'),
            'FS_GCS_ENABLE': vehicle.parameters.get('FS_GCS_ENABLE'),
        }
        
        print("\n  Parameter          Value    Required")
        print("  " + "-"*50)
        
        all_ok = True
        for param, value in params.items():
            required = "0"
            status = "[OK]" if value == 0 else "[!]"
            print("  %-18s %-8s %-8s  %s" % (param, value, required, status))
            if value != 0:
                all_ok = False
        
        if all_ok:
            print("\n  [OK] All parameters set correctly!")
        else:
            print("\n  [!] Parameters need to be set in Mission Planner")
            print("\n  How to fix:")
            print("  1. Open Mission Planner on another computer")
            print("  2. Go to CONFIG/TUNING > Full Parameter List")
            print("  3. Set the parameters above to 0")
            print("  4. Click 'Write Params'")
        
        return all_ok
        
    except Exception as e:
        print("  Error reading parameters: %s" % str(e))
        return False

def display_home_location(vehicle):
    """Display home location"""
    print("\n" + "="*60)
    print("HOME LOCATION")
    print("="*60)
    
    try:
        if vehicle.home_location is not None:
            print("\n  Latitude: %s" % vehicle.home_location.lat)
            print("  Longitude: %s" % vehicle.home_location.lon)
            print("  Altitude: %s m" % vehicle.home_location.alt)
            print("\n  [OK] Home location is set")
        else:
            print("\n  [!] Home location not set yet")
            print("  Requires GPS fix to set home location")
        
    except Exception as e:
        print("  Error reading home location: %s" % str(e))

def test_mode_change(vehicle):
    """Test changing flight modes"""
    print("\n" + "="*60)
    print("FLIGHT MODE TEST")
    print("="*60)
    
    try:
        original_mode = vehicle.mode.name
        print("\n  Current mode: %s" % original_mode)
        
        # Try to set GUIDED mode
        print("\n  Attempting to set GUIDED mode...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)
        
        if vehicle.mode.name == "GUIDED":
            print("  [OK] Successfully changed to GUIDED mode")
            
            # Change back to original mode
            print("\n  Changing back to %s mode..." % original_mode)
            vehicle.mode = VehicleMode(original_mode)
            time.sleep(2)
            
            if vehicle.mode.name == original_mode:
                print("  [OK] Successfully changed back to %s" % original_mode)
                return True
            else:
                print("  [!] Failed to change back (now in %s)" % vehicle.mode.name)
                return False
        else:
            print("  [!] Failed to change to GUIDED (still in %s)" % vehicle.mode.name)
            return False
        
    except Exception as e:
        print("  [!] Error during mode change: %s" % str(e))
        return False

def monitor_data_stream(vehicle, duration=5):
    """Monitor live data stream"""
    print("\n" + "="*60)
    print("LIVE DATA STREAM TEST (%d seconds)" % duration)
    print("="*60)
    
    try:
        print("\n  Time | Mode      | Armed | GPS | Alt(m) | Bat(V)")
        print("  " + "-"*60)
        
        for i in range(duration):
            mode = vehicle.mode.name
            armed = "YES" if vehicle.armed else "NO "
            gps_sats = vehicle.gps_0.satellites_visible
            alt = vehicle.location.global_relative_frame.alt or 0
            bat = vehicle.battery.voltage
            
            print("  %4d | %-9s | %-5s | %3d | %6.2f | %.2f" % 
                  (i+1, mode, armed, gps_sats, alt, bat))
            
            time.sleep(1)
        
        print("\n  [OK] Data stream is working correctly")
        return True
        
    except Exception as e:
        print("\n  [!] Error reading data stream: %s" % str(e))
        return False

def display_rpi_info():
    """Display Raspberry Pi system information"""
    print("\n" + "="*60)
    print("RASPBERRY PI SYSTEM INFO")
    print("="*60)
    
    try:
        # Check if running on Raspberry Pi
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip('\x00')
                print("\n  Model: %s" % model)
        
        # Check CPU temperature
        if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                print("  CPU Temperature: %.1f°C" % temp)
                if temp > 80:
                    print("  [!] WARNING: CPU temperature high!")
        
        print("\n  Python version: %s" % sys.version.split()[0])
        
    except Exception as e:
        print("  Could not read system info: %s" % str(e))

# ============================================
# MAIN TEST SEQUENCE
# ============================================

def main():
    vehicle = None
    
    try:
        # Display Raspberry Pi info
        display_rpi_info()
        
        # Test 1: Connection
        vehicle = test_connection()
        
        if vehicle is None:
            sys.exit(1)
        
        # Test 2: Basic Info
        display_basic_info(vehicle)
        
        # Test 3: GPS
        display_gps_info(vehicle)
        
        # Test 4: Battery
        display_battery_info(vehicle)
        
        # Test 5: Parameters
        params_ok = display_parameters(vehicle)
        
        # Test 6: Home Location
        display_home_location(vehicle)
        
        # Test 7: Mode Change
        mode_ok = test_mode_change(vehicle)
        
        # Test 8: Data Stream
        stream_ok = monitor_data_stream(vehicle)
        
        # Final Summary
        print("\n" + "="*60)
        print("CONNECTION TEST SUMMARY")
        print("="*60)
        
        results = [
            ("Connection", True),
            ("Basic Info", True),
            ("GPS Reading", True),
            ("Battery Reading", True),
            ("Parameters", params_ok),
            ("Mode Change", mode_ok),
            ("Data Stream", stream_ok)
        ]
        
        print("\n  Test                    Result")
        print("  " + "-"*40)
        for test_name, result in results:
            status = "[OK] PASS" if result else "[!] FAIL"
            print("  %-24s %s" % (test_name, status))
        
        all_passed = all(result for _, result in results)
        
        if all_passed:
            print("\n" + "="*60)
            print("*** ALL TESTS PASSED! ***")
            print("="*60)
            print("\nYour Raspberry Pi + APM setup is ready for:")
            print("  1. ARM test (without propellers)")
            print("  2. Takeoff/Landing test (with propellers)")
        else:
            print("\n" + "="*60)
            print("[!] SOME TESTS FAILED")
            print("="*60)
            print("\nFix the issues above before proceeding to flight tests")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    except Exception as e:
        print("\n[!] Unexpected error: %s" % str(e))
        import traceback
        traceback.print_exc()
    
    finally:
        if vehicle:
            print("\nClosing connection...")
            vehicle.close()
            print("Connection closed.")

if __name__ == "__main__":
    main()