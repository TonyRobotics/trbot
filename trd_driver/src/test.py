#!/usr/bin/python2
import sys
import time
import serial
import math
import threading

def single_thread():
    serialport_name = '/dev/ttyUSB0'
    baudrate = 115200
    conn = serial.Serial(serialport_name, baudrate)

    def get_response():
        time.sleep(0.001)
        res = ''
        while conn.inWaiting() > 0:
            res += conn.read(1)
        res = bytearray(res)
        return res

    v1 = 100
    v2 = 100
    while True:
        try:
            v1 = 100+int(10*math.sin(time.time()))
            print('===============')
            cmd = '!M {} {} \r'.format(v1, v2)

            print(cmd)
            conn.write(cmd)
            res = get_response()
            print(time.time(), res)
            time.sleep(0.1)

            #cmd = '?C \r'
            ####No Space HERE!!!!
            cmd = '?C\r'
            ####
            print(cmd)
            conn.write(cmd)
            res = get_response()
            print(time.time(), res)

            time.sleep(0.1)
        except KeyboardInterrupt:
            print('exit')
            break


class SerHandler(threading.Thread):

    def __init__(self, PORT, BAUDRATE):
        threading.Thread.__init__(self)
        self.conn = serial.Serial(serialport_name, baudrate)
        self.stop_flag = False

    def run(self):
        while not self.stop_flag:
            res = ''
            n = self.conn.inWaiting()
            if n > 0:
                res += self.conn.read(n)
            else:
                continue
            res = bytearray(res)
            print(res)
            #v = [hex(ord(r)) for r in res]
            #print(v)
            time.sleep(0.01)
            #with open('a.txt', 'a') as f:
            #    f.write(res)
    
    def set_speed(self, v1, v2):
        cmd = '!M {} {} \r'.format(v1, v2)
        self.conn.write(cmd)

    def get_encoder(self):
        cmd = '?C \r'
        self.conn.write(cmd)

    def stop(self):
        self.stop_flag = True

if __name__=='__main__':
    serialport_name = '/dev/ttyUSB0'
    baudrate = 115200

    ser_handler = SerHandler(serialport_name, baudrate)
    ser_handler.start()

    v1 = 100
    v2 = 100
    vs = [100]
    vs = [100,101,102,101]
    while True:
        try:
            for v in vs:
                v1 = v
                time.sleep(0.3)
                ser_handler.set_speed(v1, v2)
                time.sleep(0.3)
                ser_handler.get_encoder()
        except KeyboardInterrupt:
            ser_handler.stop()
            print('exited')
            break

