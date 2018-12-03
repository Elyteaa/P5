#!/usr/bin/python
import serial
import codecs
import binascii

z1baudrate = 115200
z1port = '/dev/ttyUSB0'
z1serial = serial.Serial(port=z1port, baudrate=z1baudrate, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
z1serial.timeout = None  # set read timeout
if z1serial.is_open:
    while True:
        size = z1serial.inWaiting()
        test = z1serial.read(1)
        test2 = codecs.decode(test.strip(), 'hex')
        #test2 = binascii.unhexlify(test.strip().decode('hex'))
        #print(test)
        #test2 = hex(int((test), 16))
        #test2 = hex(ord(test))
        #test2 = int(hex(test), 16)
        #test2 = int(test, 16)
        #test2 = test.int()
        #test2 = test.unpack("h", test)
        #test2 = test.encode(encoding='utf-8',errors='strict')
        print(type(test2))
        print(test2)
else:
    print('z1serial not open')
