#!usr/bin/python
import serial
import time

z1baudrate = 115200
z1port = '/dev/ttyUSB0'  # set the correct port before running it
data = []
receiveCounter = 0
receivedMessage = [None]*255

z1serial = serial.Serial(port=z1port, baudrate=z1baudrate, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)

z1serial.close()