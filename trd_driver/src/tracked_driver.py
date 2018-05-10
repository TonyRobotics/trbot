#!/usr/bin/python

import sys
import time
import math
import struct
from threading import Lock

import serial
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

lock = Lock()

class TrackedSerialDriver():

    def __init__(self, serialport_name, baudrate):
        self.conn = serial.Serial(serialport_name, baudrate)
        self.encoder1_offset = 0
        self.encoder2_offset = 0
        self.first_time_flag = True

    def send_cmd(self, cmd):
        time.sleep(0.1)
        print(cmd)
        self.conn.write(cmd)

    def get_response(self):
        time.sleep(0.1)
        res = ''
        while self.conn.inWaiting() > 0:
            res += self.conn.read(1)
        res = bytearray(res)
        print(time.time(), res)
        return res

    def set_speed(self, v1, v2):
        cmd = '!M {} {} \r'.format(v1, v2)
        lock.acquire()
        self.send_cmd(cmd)
        res = self.get_response()
        #print(time.time(), res)
        lock.release()

    def get_encoders(self):
        cmd = '?C\r'
        lock.acquire()
        self.send_cmd(cmd)
        res = self.get_response()
        lock.release()
        #print(time.time(), res)
        return (0,0)
        encoder1, = struct.unpack('>i', res[3:7])
        encoder2, = struct.unpack('>i', res[7:11])
        if self.first_time_flag:
            self.encoder1_offset = encoder1
            self.encoder2_offset = encoder2
            self.first_time_flag = False
        encoder1 -= self.encoder1_offset
        encoder2 -= self.encoder2_offset
        print('{}: {} {}'.format(time.time(), encoder1, encoder2))
        return (encoder1, encoder2)

class DriverNode():

    def __init__(self, node_name, serialport_name, baudrate):
        self.serial_driver = TrackedSerialDriver(serialport_name, baudrate)
        rospy.init_node(node_name)
        self.linear_coef = 82
        self.angular_coef = 14.64
        self.left_coef = 5
        self.right_coef = -5
        self.encoder_ticks_per_rev = 15*2500
        self.base_width = 0.53
        self.wheel_diameter = 0.16
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.encoder1_prev = 0
        self.encoder2_prev = 0
        self.time_prev = rospy.Time.now()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.v1 = 0
        self.v2 = 0

    def vel_callback(self, vel_msg):
        self.v1 = self.linear_coef * vel_msg.linear.x
        self.v2 = self.linear_coef * vel_msg.linear.x
        self.v1 -= self.angular_coef * vel_msg.angular.z
        self.v2 += self.angular_coef * vel_msg.angular.z
        self.v1 *= self.left_coef
        self.v2 *= self.right_coef
        self.v1 = int(self.v1) if self.v1<1000 else 1000
        self.v2 = int(self.v2) if self.v2<1000 else 1000
        self.v1 = int(self.v1) if self.v1>-1000 else -1000
        self.v2 = int(self.v2) if self.v2>-1000 else -1000

    def update_odom(self):
        (encoder1, encoder2) = self.serial_driver.get_encoders()
        time_current = rospy.Time.now()
        time_elapsed = (time_current - self.time_prev).to_sec()
        self.time_prev = time_current
        dleft = self.left_coef * math.pi * self.wheel_diameter * \
                (encoder1 - self.encoder1_prev) / self.encoder_ticks_per_rev
        dright = self.right_coef * math.pi * self.wheel_diameter * \
                (encoder2 - self.encoder2_prev) / self.encoder_ticks_per_rev
        self.encoder1_prev = encoder1
        self.encoder2_prev = encoder2
        d = (dleft + dright) / 2
        dtheta = (dright - dleft) / self.base_width
        if d != 0:
            dx = math.cos(dtheta) * d
            dy = -math.sin(dtheta) * d
            self.x += dx*math.cos(self.theta)-dy*math.sin(self.theta)
            self.y += dx*math.sin(self.theta)+dy*math.cos(self.theta)
        self.theta += dtheta

        self.odom.header.stamp = time_current
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        self.odom.twist.twist.linear.x = d / time_elapsed
        self.odom.twist.twist.angular.z = dtheta / time_elapsed

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                print('=======')
                self.update_odom()
                time.sleep(0.1)
                print('--------')
                self.pub_odom.publish(self.odom)
                self.serial_driver.set_speed(self.v1, self.v2)
                print('--------')
                time.sleep(0.1)
                #self.tf_broadcaster.sendTransform((self.x,self.y,0),
                #    tf.transformations.quaternion_from_euler(0, 0, self.theta),
                #    rospy.Time.now(),
                #    'base_link',
                #    'odom')
                #rate.sleep()
            except KeyboardInterrupt:
                print('exit.')
                break
            #except serial.serialutil.SerialException:
            #    print('exit serial.')
            #    break

if __name__=='__main__':
    serialport_name = '/dev/ttyUSB0'
    baudrate = 115200
    driver_node = DriverNode('tracked_driver', serialport_name, baudrate)
    driver_node.run()

