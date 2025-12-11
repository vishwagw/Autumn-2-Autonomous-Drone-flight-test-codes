from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect via USB instead of serial
vehicle = connect('/dev/ttyUSB0', baud=115200, wait_ready=True)
# Or it might be /dev/ttyACM0 depending on your system

# Rest of the code remains the same
def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    
    # ... rest of code
