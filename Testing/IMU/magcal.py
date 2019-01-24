#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
import math 
import numpy as np
import time
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

imu = InertialMeasurementUnit(bus = "RPI_1") #RPI_1 GPG3_AD1

x = []
y = []
z = []
corr_x = []
corr_y = []
corr_z = []
i = 0
j = 0

while j < 10:
    mag = imu.read_magnetometer()
    x.append(mag[0])
    y.append(mag[1])
    z.append(mag[2])
    j += 1
del x[0]
del y[0]
del z[0]


while i < 1500:
    mag   = imu.read_magnetometer()
    x.append(mag[0])
    y.append(mag[1])
    z.append(mag[2])
    i += 1
    


    offset_x = (max(x) + min(x)) / 2
    offset_y = (max(y) + min(y)) / 2
    offset_z = (max(z) + min(z)) / 2

    avg_delta_x = (max(x) - min(x)) / 2
    avg_delta_y = (max(y) - min(y)) / 2
    avg_delta_z = (max(z) - min(z)) / 2

    avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

    scale_x = avg_delta / avg_delta_x
    scale_y = avg_delta / avg_delta_y
    scale_z = avg_delta / avg_delta_z
#while j < 1000:
    #mag   = imu.read_magnetometer()
    corrected_x = (mag[0] - offset_x) * scale_x
    corrected_y = (mag[1] - offset_y) * scale_y
    corrected_z = (mag[2] - offset_z) * scale_z
    
    corr_x.append(corrected_x)
    corr_y.append(corrected_y)
    corr_z.append(corrected_z)
    
    #j += 1
print('x =', corr_x)
print('y =', corr_y)
print('z =', corr_z)
    
    
#print('x = ', corrected_x)
#print('y = ', corrected_y)
#print('z = ', corrected_z)

    #corr_x.append(corrected_x)
    #corr_y.append(corrected_y)
    #corr_z.append(corrected_z)
    
#del corr_x[0]
#del corr_y[0]
#del corr_z[0]..
