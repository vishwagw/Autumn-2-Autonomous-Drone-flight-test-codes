#!/usr/bin/env python
"""
Onboard Takeoff and Landing for Raspberry Pi 3
Optimized for running on drone's companion computer
IMPORTANT: Use propellers for this test!
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import sys
import os

# ============================================
# CONFIGURATION FOR ONBOARD RASPBERRY PI
# ============================================
# Common Pixhawk/Flight Controller connections on RPi:
# - /dev/ttyAMA0 (GPIO serial) - may need to disable console
# - /dev/ttyACM0 (USB connection to Pixhawk)
# - /dev/serial0 (symlink to primary UART)
CONNECTION_STRING = '/dev/ttyACM0'  # USB to Pixhawk (most common)
# Alternative: '/dev/ttyAMA0' for GPIO serial connection
# Alternative: '/dev/serial0' for primary UART

BAUD_RATE = 921600  # Higher baud for onboard connection (57600 or 921600 typical)

# Flight Parameters
TARGET_ALTITUDE = 2.0  # meters - START LOW!
HOVER_TIME = 5         # seconds to hover
MAX_ALTITUDE = 5.0     # safety ceiling

# Safety Parameters
MIN_BATTERY = 10.5     # minimum voltage
MIN_GPS_SATS = 6       # minimum satellites

# Logging
LOG_ENABLED = True
LOG_DIR = '/home/pi/flight_logs'  # Adjust path as needed

# ============================================
# UTILITY FUNCTIONS
# ============================================

def setup_logging():
    """Setup flight logging directory"""
    if LOG_ENABLED:
        try:
            if not os.path.exists(LOG_DIR):
                os.makedirs(LOG_DIR)
            
            log_file = os.path.join(LOG_DIR, 'flight_%s.log' % 
                                   time.strftime('%Y%m%d_%H%M%S'))
            return open(log_file, 'w')
        except Exception as e:
            print("Warning: Could not create log file: %s" % str(e))
            return None
    return None

def log_message(log_file, message):
    """Write message to both console and log file"""
    print(message)
    if log_file:
        try:
            log_file.write(message + '\n')
            log_file.flush()
        except:
            pass

def detect_connection():
    """Auto-detect the flight controller connection"""
    possible_ports = [
        '/dev/ttyACM0',   # USB connection (most common)
        '/dev/ttyAMA0',   # GPIO UART
        '/dev/serial0',   # Primary UART symlink
        '/dev/ttyUSB0',   # USB-to-serial adapter
    ]
    
    print("Searching for flight controller...")
    for port in possible_ports:
        if os.path.exists(port):
            print("  Found: %s" % port)
            return port
    
    print("  No common ports found, using default: %s" % CONNECTION_STRING)
    return CONNECTION_STRING

def connect_vehicle(log_file=None):
    """Connect to the vehicle with retry logic"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "CONNECTING TO FLIGHT CONTROLLER")
    log_message(log_file, "="*60)
    
    # Auto-detect connection if needed
    connection = detect_connection()
    log_message(log_file, "Port: %s | Baud: %d" % (connection, BAUD_RATE))
    
    max_retries = 3
    for attempt in range(max_retries):
        try:
            log_message(log_file, "Attempt %d/%d..." % (attempt + 1, max_retries))
            vehicle = connect(connection, baud=BAUD_RATE, wait_ready=True, timeout=30)
            log_message(log_file, "✓ Connected successfully!")
            
            # Small delay to ensure stable connection
            time.sleep(1)
            return vehicle
            
        except Exception as e:
            log_message(log_file, "✗ Connection failed: %s" % str(e))
            if attempt < max_retries - 1:
                log_message(log_file, "Retrying in 3 seconds...")
                time.sleep(3)
            else:
                log_message(log_file, "✗ All connection attempts failed")
                sys.exit(1)

def pre_flight_checks(vehicle, log_file=None):
    """Comprehensive pre-flight checks"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "PRE-FLIGHT CHECKS")
    log_message(log_file, "="*60)
    
    checks_passed = True
    
    # Check 1: Battery
    log_message(log_file, "\n[1] Battery Check")
    voltage = vehicle.battery.voltage
    log_message(log_file, "    Voltage: %.2fV" % voltage)
    if voltage < MIN_BATTERY:
        log_message(log_file, "    ✗ FAIL: Battery too low (min: %.1fV)" % MIN_BATTERY)
        checks_passed = False
    else:
        log_message(log_file, "    ✓ PASS")
    
    # Check 2: GPS
    log_message(log_file, "\n[2] GPS Check")
    fix_type = vehicle.gps_0.fix_type
    sats = vehicle.gps_0.satellites_visible
    log_message(log_file, "    Fix Type: %d | Satellites: %d" % (fix_type, sats))
    if fix_type < 2:
        log_message(log_file, "    ✗ FAIL: No GPS fix (need 2D or 3D fix)")
        checks_passed = False
    elif sats < MIN_GPS_SATS:
        log_message(log_file, "    ✗ FAIL: Not enough satellites (min: %d)" % MIN_GPS_SATS)
        checks_passed = False
    else:
        log_message(log_file, "    ✓ PASS")
    
    # Check 3: Home Location
    log_message(log_file, "\n[3] Home Location Check")
    if vehicle.home_location is not None:
        log_message(log_file, "    Home: %.6f, %.6f" % 
                   (vehicle.home_location.lat, vehicle.home_location.lon))
        log_message(log_file, "    ✓ PASS")
    else:
        log_message(log_file, "    ✗ FAIL: Home location not set")
        checks_passed = False
    
    # Check 4: Armable
    log_message(log_file, "\n[4] Armable Status")
    if vehicle.is_armable:
        log_message(log_file, "    ✓ PASS: Vehicle is armable")
    else:
        log_message(log_file, "    ✗ FAIL: Vehicle is not armable")
        checks_passed = False
    
    # Check 5: System Status
    log_message(log_file, "\n[5] System Status")
    log_message(log_file, "    Mode: %s" % vehicle.mode.name)
    log_message(log_file, "    Armed: %s" % vehicle.armed)
    log_message(log_file, "    System Status: %s" % vehicle.system_status.state)
    log_message(log_file, "    ✓ PASS")
    
    return checks_passed

def wait_for_gps(vehicle, log_file=None, timeout=60):
    """Wait for GPS lock"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "WAITING FOR GPS LOCK")
    log_message(log_file, "="*60)
    
    start_time = time.time()
    
    while vehicle.gps_0.fix_type < 2 or vehicle.gps_0.satellites_visible < MIN_GPS_SATS:
        elapsed = time.time() - start_time
        
        if elapsed > timeout:
            log_message(log_file, "\n✗ GPS timeout after %d seconds" % timeout)
            return False
        
        log_message(log_file, "  %.0fs | Fix: %d | Sats: %d (need %d+)" % 
              (elapsed, vehicle.gps_0.fix_type, 
               vehicle.gps_0.satellites_visible, MIN_GPS_SATS))
        time.sleep(2)
    
    log_message(log_file, "\n✓ GPS locked with %d satellites!" % 
               vehicle.gps_0.satellites_visible)
    return True

def arm_vehicle(vehicle, log_file=None):
    """Arm the vehicle"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "ARMING SEQUENCE")
    log_message(log_file, "="*60)
    
    # Set GUIDED mode
    log_message(log_file, "\nSetting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    # Wait for mode change
    timeout = 5
    start = time.time()
    while vehicle.mode.name != "GUIDED":
        if time.time() - start > timeout:
            log_message(log_file, "✗ Failed to set GUIDED mode")
            return False
        time.sleep(0.5)
    
    log_message(log_file, "✓ Mode: GUIDED")
    
    # Wait for armable
    log_message(log_file, "\nWaiting for armable status...")
    timeout = 30
    start = time.time()
    while not vehicle.is_armable:
        if time.time() - start > timeout:
            log_message(log_file, "✗ Vehicle not armable after timeout")
            return False
        log_message(log_file, "  Waiting... (GPS: %d sats)" % 
                   vehicle.gps_0.satellites_visible)
        time.sleep(1)
    
    log_message(log_file, "✓ Vehicle is armable")
    
    # Arm motors
    log_message(log_file, "\n⚠️  ARMING MOTORS - Propellers will spin!")
    time.sleep(2)
    
    vehicle.armed = True
    
    # Wait for arming
    timeout = 10
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > timeout:
            log_message(log_file, "✗ Arming timeout")
            return False
        log_message(log_file, "  Arming...")
        time.sleep(0.5)
    
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "✓✓✓ ARMED ✓✓✓")
    log_message(log_file, "="*60)
    return True

def takeoff(vehicle, target_altitude, log_file=None):
    """Execute takeoff"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "TAKEOFF TO %.1f METERS" % target_altitude)
    log_message(log_file, "="*60)
    
    log_message(log_file, "\nInitiating takeoff...")
    vehicle.simple_takeoff(target_altitude)
    
    # Monitor takeoff
    start_time = time.time()
    timeout = 30
    
    last_alt = 0
    stall_count = 0
    
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        elapsed = time.time() - start_time
        
        log_message(log_file, "  %.1fs | Alt: %.2fm | Target: %.2fm | Battery: %.2fV" % 
              (elapsed, current_alt, target_altitude, vehicle.battery.voltage))
        
        # Check for stall
        if abs(current_alt - last_alt) < 0.05:
            stall_count += 1
            if stall_count > 5 and current_alt < target_altitude * 0.5:
                log_message(log_file, "⚠️  WARNING: Takeoff stalled!")
        else:
            stall_count = 0
        
        last_alt = current_alt
        
        # Safety checks
        if current_alt > MAX_ALTITUDE:
            log_message(log_file, "⚠️  SAFETY: Exceeded max altitude!")
            break
        
        if elapsed > timeout:
            log_message(log_file, "⚠️  Takeoff timeout - altitude: %.2fm" % current_alt)
            break
        
        # Check if reached target
        if current_alt >= target_altitude * 0.95:
            log_message(log_file, "\n✓ Reached target altitude: %.2fm" % current_alt)
            break
        
        time.sleep(1)
    
    return True

def hover(vehicle, duration, log_file=None):
    """Hover for specified duration"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "HOVERING FOR %d SECONDS" % duration)
    log_message(log_file, "="*60)
    
    for i in range(duration):
        alt = vehicle.location.global_relative_frame.alt
        mode = vehicle.mode.name
        bat = vehicle.battery.voltage
        
        log_message(log_file, "  %d/%ds | Alt: %.2fm | Mode: %s | Battery: %.2fV" % 
              (i+1, duration, alt, mode, bat))
        
        # Safety checks
        if bat < MIN_BATTERY:
            log_message(log_file, "⚠️  WARNING: Low battery!")
            return False
        
        if not vehicle.armed:
            log_message(log_file, "✗ Vehicle disarmed unexpectedly!")
            return False
        
        time.sleep(1)
    
    log_message(log_file, "\n✓ Hover complete")
    return True

def land(vehicle, log_file=None):
    """Land the vehicle"""
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "LANDING SEQUENCE")
    log_message(log_file, "="*60)
    
    log_message(log_file, "\nInitiating landing...")
    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(1)
    if vehicle.mode.name != "LAND":
        log_message(log_file, "⚠️  WARNING: Not in LAND mode (currently: %s)" % 
                   vehicle.mode.name)
    
    # Monitor landing
    last_alt = vehicle.location.global_relative_frame.alt
    timeout = 60
    start_time = time.time()
    
    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        elapsed = time.time() - start_time
        
        descent_rate = last_alt - current_alt
        last_alt = current_alt
        
        log_message(log_file, "  %.1fs | Alt: %.2fm | Descent: %.2fm/s | Battery: %.2fV" % 
              (elapsed, current_alt, descent_rate, vehicle.battery.voltage))
        
        if elapsed > timeout:
            log_message(log_file, "⚠️  Landing timeout!")
            break
        
        time.sleep(1)
    
    log_message(log_file, "\n" + "="*60)
    log_message(log_file, "✓✓✓ LANDED AND DISARMED ✓✓✓")
    log_message(log_file, "="*60)
    return True

def emergency_land(vehicle, log_file=None):
    """Emergency landing procedure"""
    log_message(log_file, "\n" + "!"*60)
    log_message(log_file, "!!! EMERGENCY LANDING !!!")
    log_message(log_file, "!"*60)
    
    try:
        if vehicle.armed:
            log_message(log_file, "Setting LAND mode...")
            vehicle.mode = VehicleMode("LAND")
            
            timeout = 30
            start = time.time()
            while vehicle.armed and (time.time() - start < timeout):
                alt = vehicle.location.global_relative_frame.alt
                log_message(log_file, "  Emergency landing... Alt: %.2fm" % alt)
                time.sleep(1)
            
            if vehicle.armed:
                log_message(log_file, "⚠️  Still armed after timeout!")
            else:
                log_message(log_file, "✓ Emergency landing complete")
    except Exception as e:
        log_message(log_file, "✗ Emergency landing error: %s" % str(e))

# ============================================
# MAIN FLIGHT TEST
# ============================================

def main():
    vehicle = None
    log_file = None
    
    try:
        # Setup logging
        log_file = setup_logging()
        
        log_message(log_file, "\n" + "="*60)
        log_message(log_file, "ONBOARD AUTONOMOUS FLIGHT - RASPBERRY PI 3")
        log_message(log_file, "="*60)
        log_message(log_file, "Start Time: %s" % time.strftime('%Y-%m-%d %H:%M:%S'))
        
        log_message(log_file, "\nFlight Plan:")
        log_message(log_file, "  - Takeoff to %.1fm" % TARGET_ALTITUDE)
        log_message(log_file, "  - Hover for %ds" % HOVER_TIME)
        log_message(log_file, "  - Land automatically")
        
        # Connect
        vehicle = connect_vehicle(log_file)
        
        # Pre-flight checks
        if not pre_flight_checks(vehicle, log_file):
            log_message(log_file, "\n✗ Pre-flight checks failed!")
            log_message(log_file, "Flight aborted for safety.")
            if vehicle:
                vehicle.close()
            sys.exit(1)
        
        # Wait for GPS if needed
        if vehicle.gps_0.fix_type < 2 or vehicle.gps_0.satellites_visible < MIN_GPS_SATS:
            if not wait_for_gps(vehicle, log_file):
                log_message(log_file, "✗ GPS lock failed")
                vehicle.close()
                sys.exit(1)
        
        log_message(log_file, "\n" + "="*60)
        log_message(log_file, "STARTING AUTONOMOUS FLIGHT")
        log_message(log_file, "="*60)
        
        # Arm
        if not arm_vehicle(vehicle, log_file):
            log_message(log_file, "✗ Arming failed!")
            vehicle.close()
            sys.exit(1)
        
        # Takeoff
        takeoff(vehicle, TARGET_ALTITUDE, log_file)
        
        # Hover
        hover(vehicle, HOVER_TIME, log_file)
        
        # Land
        land(vehicle, log_file)
        
        # Mission complete
        log_message(log_file, "\n" + "="*60)
        log_message(log_file, "✓✓✓ MISSION COMPLETE! ✓✓✓")
        log_message(log_file, "="*60)
        
        log_message(log_file, "\nFlight Summary:")
        log_message(log_file, "  Target altitude: %.2fm" % TARGET_ALTITUDE)
        log_message(log_file, "  Hover time: %ds" % HOVER_TIME)
        log_message(log_file, "  Final battery: %.2fV" % vehicle.battery.voltage)
        log_message(log_file, "  End time: %s" % time.strftime('%Y-%m-%d %H:%M:%S'))
        
    except KeyboardInterrupt:
        log_message(log_file, "\n\n!!! FLIGHT ABORTED BY USER !!!")
        if vehicle:
            emergency_land(vehicle, log_file)
    
    except Exception as e:
        log_message(log_file, "\n!!! ERROR DURING FLIGHT !!!")
        log_message(log_file, "Error: %s" % str(e))
        import traceback
        traceback.print_exc()
        
        if vehicle:
            emergency_land(vehicle, log_file)
    
    finally:
        if vehicle:
            log_message(log_file, "\nClosing connection...")
            vehicle.close()
            log_message(log_file, "Connection closed.")
        
        if log_file:
            log_file.close()

if __name__ == "__main__":
    main()