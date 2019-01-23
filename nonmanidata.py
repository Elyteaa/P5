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
i = 0

while i < 5000:
    mag = imu.read_magnetometer()
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
#for j in range(len(x)+1):
#    corr_x = np.array([x] - offset_x)
#    corr_y = y - offset_y
#    corr_z = z - offset_z
    #i += 1
#del x[0]
#del y[0]
#del z[0]

x[:] = [(a - offset_x) * scale_x for a in x]
y[:] = [(b - offset_y) * scale_y for b in y]
z[:] = [(c - offset_z) * scale_z for c in z]


print(x)
print(y)
print(z)
    