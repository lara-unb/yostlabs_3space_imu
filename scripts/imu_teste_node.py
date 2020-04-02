#!/usr/bin/env python

import rospy

from std_msgs.msg import Int8
from std_msgs.msg import String
import serial
import binascii
import ema.libs.yei.threespace_api as ts_api
import time


def teste():
    rospy.init_node('imu', anonymous=False)
    pub = rospy.Publisher('imu/data', String, queue_size=10)


    number_of_sensors = 1
    p = False # print?
    command = 0

    addresses = [7,8,2,3,4,5,1,6]

    portIMU = '/dev/ttyACM0'
    serial_port = serial.Serial(port=portIMU, baudrate=115200, timeout=0.001)
    time.sleep(0.1)
    serial_port.flush()
    time.sleep(0.1)

    # Set streaming slots
    for i in range(len(addresses)):
        msg = '>'+str(addresses[i])+',80,'+str(command)+',255,255,255,255,255,255,255\n'
        print(msg)
        serial_port.write(msg)
        time.sleep(0.1)
        out = ''
        while serial_port.inWaiting():
            out += '>> ' + serial_port.read(serial_port.inWaiting())
        print(out)
    out = ''

    # Start streaming
    for i in range(len(addresses)):
        serial_port.write('>'+str(addresses[i])+',85\n')
        time.sleep(0.1)
        while serial_port.inWaiting():
            out = '>> ' + serial_port.read(serial_port.inWaiting())

    print('Start')


    # define loop rate (in hz)
    rate = rospy.Rate(200)



    # node loop
    while not rospy.is_shutdown():

        bytes_to_read = serial_port.inWaiting()
        if bytes_to_read > 0:
            data = serial_port.read(bytes_to_read)
            id_str = binascii.hexlify(data[1].encode('utf-8'))
            if p:
                print(id_str)

            try:
                id = int(id_str)
                pub.publish(str(id))
            except ValueError:
                print('NAN')
                pub.publish('NAN')

        
        # sleep until it's time to work again
        rate.sleep()


if __name__ == '__main__':
    try:
        teste()
    except rospy.ROSInterruptException:
        pass

