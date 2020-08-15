# -*- coding: utf-8 -*-
"""
Created on Tue Jul  2 01:45:58 2019

@author: shank
"""

import time
import board
import busio
import adafruit_gps
import math
from math import asin,atan,cos,sin,pi
import adafruit_bno055
import logging
import sys
from adafruit_bno055 import BNO055


# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins
RX = board.RX
TX = board.TX

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
uart = busio.UART(TX, RX, baudrate=10000, timeout=3000)

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)
# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
# https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
# Turn on the basic GGA and RMC info (what you typically want)
# Turn on just minimum info (RMC only, location):
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b'PMTK220,16000')      #here setted to 160HZ
#You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing. This would be twice a second (2hz, 500ms delay):
gps.send_command(b'PMTK220,500')
# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    # Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)
    
    
    '''  # Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')'''
    
    
while True:
    # Make sure to call gps.update() every loop iteration and at least twice
# as fast as data comes from the GPS unit (usually every second).
# This returns a bool that's true if it parsed new data (you can ignore it
# though if you don't care and instead look at the has_fix property).
    gps.update()
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        # Try again if we don't have a fix yet
    if not gps.has_fix:
        print('Waiting for fix...')
        continue
    # We have a fix! (gps.has_fix is true)
# Print out details about the fix like location, date, etc.
#print('=' * 40) # Print a separator line.
#print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
#gps.timestamp_utc.tm_mon, # Grab parts of the time from the
#gps.timestamp_utc.tm_mday, # struct_time object that holds
#gps.timestamp_utc.tm_year, # the fix time. Note you might
#gps.timestamp_utc.tm_hour, # not get all data like year, day,
#gps.timestamp_utc.tm_min, # month!
#gps.timestamp_utc.tm_sec))
        while True:
            print('Latitude: {0:.6f} degrees'.format(gps.latitude))
            print('Longitude: {0:.6f} degrees'.format(gps.longitude))
            R=6731000
            latX=R*math.cos(gps.latitude)*math.cos(gps.longitude)
            longx=R*math.cos(gps.latitude)*math.sin(gps.longitude)
            print('X: {0:.6f} degrees'.latx)
            print('Y: {0:.6f} degrees'.longx)
            #print('Fix quality: {}'.format(gps.fix_quality))
# Some attributes beyond latitude, longitude and timestamp are optional
# and might not be present. Check if they're None before trying to use!
            if gps.satellites is not None:
                print('# satellites: {}'.format(gps.satellites))
                      #if gps.altitude_m is not None:
#print('Altitude: {} meters'.format(gps.altitude_m))
#if gps.speed_knots is not None:
#print('Speed: {} knots'.format(gps.speed_knots))
            if gps.track_angle_deg is not None:
                print('Track angle: {} degrees'.format(gps.track_angle_deg))
            if gps.horizontal_dilution is not None:
                print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
                #if gps.height_geoid is not None:
#print('Height geo ID: {} meters'.format(gps.height_geoid))
                break
            
# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    
    '''# Display the floating point data 
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);'''
    
    sys, gyro, accel, mag = bno.get_calibration_status()
    x,y,z,w = bno.read_quaterion()
    print(x)
    print(y)
    print(z)
    Y=y*y;
    roll=math.atan2(2*((w*x) +(y*z))/ (1 -(2*(x*x + Y))));
    pitch = math.asin(2*(w*y-x*z));
    yaw = math.atan2(2*(w*z)+(x*y)/(1- (2*(Y+z*z))));
    rollDeg  = 57.2958 * roll;
    pitchDeg = 57.2958 * pitch;
    yawDeg   = 57.2958 * yaw;
    print(rollDeg);
    print(pitchDeg);
    print(yawDeg);
    time.sleep(1)
