from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect via USB instead of serial
vehicle = connect('/dev/ttyUSB0', baud=115200, wait_ready=True)
# Or it might be /dev/ttyACM0 depending on your system

# Rest of the code remains the same
