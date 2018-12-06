#!/usr/bin/env python
#
# https://www.dexterindustries.com
#
# Copyright (c) 2017 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information see https://github.com/DexterInd/DI_Sensors/blob/master/LICENSE.md
#
# Python example program for the Dexter Industries IMU Sensor

from __future__ import print_function
from __future__ import division
import math 
import numpy
import time
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

print("Example program for reading a Dexter Industries IMU Sensor on a GoPiGo3 AD1 port.")

imu = InertialMeasurementUnit(bus = "GPG3_AD1") #RPI_1 GPG3_AD1

while True:
    # Read the magnetometer, gyroscope, accelerometer, euler, and temperature values
    mag   = imu.read_magnetometer()
    gyro  = imu.read_gyroscope()
    accel = imu.read_accelerometer()
    euler = imu.read_euler()
    temp  = imu.read_temperature()

    string_to_print = "Magnetometer X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                      "Gyroscope X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                      "Accelerometer X: {:.1f}  Y: {:.1f} Z: {:.1f} " \
                      "Euler Heading: {:.1f}  Roll: {:.1f}  Pitch: {:.1f} " \
                      "Temperature: {:.1f}C".format(mag[0], mag[1], mag[2],
                                                    gyro[0], gyro[1], gyro[2],
                                                    accel[0], accel[1], accel[2],
                                                    euler[0], euler[1], euler[2],
                                                    temp)
    
    #angle = math.atan2(accel[1], accel[0])
    #xyu = math.degrees(angle)
    #print(string_to_print)
    #print("angle first = ", xyu)
    #anglez = (180/math.pi * math.atan2(accel[0], accel[1]) % 360)
    #print("angle z = ", anglez)
    angley = (180/math.pi * math.atan2(mag[0], mag[2]) % 360)
    print("angle y = ", angley)
    #anglex = (180/math.pi * math.atan2(accel[1], accel[2]) % 360)
    #print("angle x = ", anglex)
    #angle1 = (180/math.pi * math.atan2(accel[1], accel[0]) % 360)
    #angle1 = (180/math.pi * math.atan2(accel[1], accel[0]) % 360)
    #print("angle 2 = ", angle1)
