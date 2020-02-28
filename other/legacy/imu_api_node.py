#!/usr/bin/env python

import rospy

from std_msgs.msg import Int8
from std_msgs.msg import String

import ema.libs.yei.threespace_api as ts_api
import time


def teste():
    rospy.init_node('imu', anonymous=False)
    pub = rospy.Publisher('imu/data', String, queue_size=10)




    ################################################################################
    ################ Second getting data over a wireless connection ################
    ################################################################################
    #device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_DNG)


    ## Only one 3-Space Sensor Dongle device is needed so we are just going to
    ## take the first one from the list.
    #com_port = device_list[0]
    dng_device = ts_api.TSDongle(com_port='/dev/ttyACM0')

    ## If a connection to the COM port fails, None is returned.
    ## Now we need to get our Wireless device from our Dongle device.
    ## Indexing into the TSDongle instance like it was a list will return a
    ## TSWLSensor instance.
    wl_device = dng_device[7]

    ## Set the stream slots for getting the tared orientation of the device as a
    ## quaternion, the raw component data, and the button state
    wl_device.setStreamingSlots(slot0='getTaredOrientationAsQuaternion')

    ## Now we can start getting the streaming batch data from the device.
    print("==================================================")
    print("Getting the streaming batch data.")


    # define loop rate (in hz)
    rate = rospy.Rate(200)



    # node loop
    while not rospy.is_shutdown():

        L = wl_device.getStreamingBatch()
        # print(L)
        if L is not None:
            pub.publish(" ".join(str(x) for x in L))
            # print("=======================================\n")

        # sleep until it's time to work again
        rate.sleep()


if __name__ == '__main__':
    try:
        teste()
    except rospy.ROSInterruptException:
        pass

