#!/usr/bin/python2

import sys
import time
import serial

if __name__=='__main__':
    serialport_name = '/dev/ttyUSB0'
    baudrate = 115200
    conn = serial.Serial(serialport_name, baudrate)

    def get_response():
        time.sleep(0.01)
        res = ''
        while conn.inWaiting() > 0:
            res += conn.read(1)
        res = bytearray(res)
        return res

    while True:
        try:
            cmd = '!M 100 100 \r'
            print(cmd)
            conn.write(cmd)
            res = get_response()
            print(time.time(), res)

            cmd = '?C \r'
            print(cmd)
            conn.write(cmd)
            res = get_response()
            print(time.time(), res)

            time.sleep(0.1)
        except KeyboardInterrupt:
            print('exit')
            break

