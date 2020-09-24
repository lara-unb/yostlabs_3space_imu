#!/usr/bin/env python

"""

Particularly, this code initializes the IMU devices, receives the measurements 
and publish them as ROS messages.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

                    _________ NOTES _________

To run with Python3, change shebang for "!/usr/bin/env python3" and uncomment
the first imports from future and builtins. It has been tested with ROS
Noetic on the Raspberry Pi 4.

"""

# # Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
import modules.imu as imu

# Import ROS msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from ema_common_msgs.srv import SetUInt16

# Import utilities
import yaml
import rospkg
import numpy

def kill_node_callback(req):
    """ROS Service handler to shutdown this node.

    Attributes:
        req (Empty): empty input
    """
    # Shutdown this node and rely on roslaunch respawn to restart
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
    return {}

def set_imu_number_callback(req):
    """ROS Service handler to set a different IMU number.

    Attributes:
        req (int): new IMU number from 0 to 10
    """
    rospy.loginfo('Set imu number: service request')
    imu_now = rospy.get_param('imu/wireless_id/pedal')
    msg = str(imu_now)
    if imu_now != req.data:
        if req.data > 0 and req.data <= 10:  # Acceptable range
            rospy.set_param('imu/wireless_id/pedal', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            imu_cfg_path = rospack.get_path('yostlabs_3space_imu')+'/config/imu.yaml'
            # Change the config yaml file
            with open(imu_cfg_path, 'r') as f:
                imu_file = yaml.safe_load(f)
                imu_file['wireless_id']['pedal'] = req.data
            with open(imu_cfg_path, 'w') as f:
                yaml.safe_dump(imu_file, f)
            # Shutdown this node and rely on roslaunch respawn to restart
            msg = str(req.data)
            rospy.loginfo('Node shutdown: new imu number')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def main():
    # Init imu node
    rospy.loginfo('Initializing node')
    rospy.init_node('imu')

    # Get imu config
    rospy.loginfo('Building manager class')
    imu_manager = imu.IMU(rospy.get_param('imu'))

    # Prepare function to be executed when shutting down
    rospy.on_shutdown(imu_manager.shutdown)

    # List provided services
    rospy.loginfo('Setting up services')
    services = {}
    services['kill_node'] = rospy.Service('imu/kill_node',
        Empty, kill_node_callback)
    services['set_imu_number'] = rospy.Service('imu/set_imu_number',
        SetUInt16, set_imu_number_callback)

    # List published topics
    rospy.loginfo('Setting up topics')
    pub = {}
    for name in imu_manager.imus:
        pub[name] = rospy.Publisher('imu/'+name, Imu, queue_size=10)
        pub[name+'_batteryVoltage'] = rospy.Publisher('imu/'+name+'_batteryVoltage', Float64, queue_size=10)
        pub[name+'_batteryPercent'] = rospy.Publisher('imu/'+name+'_batteryPercent', UInt8, queue_size=10)
        pub[name+'_buttons'] = rospy.Publisher('imu/'+name+'_buttons', UInt8, queue_size=10)

    # Define loop rate (in hz)
    rate = rospy.Rate(200)

    # Node loop
    while not rospy.is_shutdown():
        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            # Pooling mode
            if imu_manager.streaming == False:
                # Messages are shared by all imus
                imuMsg = Imu()
                imuMsg.header.stamp = timestamp
                imuMsg.header.frame_id = frame_id
                buttons = UInt8()

                for name in imu_manager.imus:
                    orientation = imu_manager.getQuaternion(name)
                    rospy.logdebug('Orientation X Y Z W: %s', orientation)

                    imuMsg.orientation.x = orientation[0]
                    imuMsg.orientation.y = orientation[1]
                    imuMsg.orientation.z = orientation[2]
                    imuMsg.orientation.w = orientation[3]

                    angular_velocity = imu_manager.getGyroData(name)
                    rospy.logdebug('Angular speed X Y Z: %s', angular_velocity)

                    imuMsg.angular_velocity.x = angular_velocity[0]
                    imuMsg.angular_velocity.y = angular_velocity[1]
                    imuMsg.angular_velocity.z = angular_velocity[2]

                    linear_acceleration = imu_manager.getAccelData(name)
                    rospy.logdebug('Linear acceleration X Y Z: %s', linear_acceleration)

                    imuMsg.linear_acceleration.x = -linear_acceleration[0]
                    imuMsg.linear_acceleration.y = -linear_acceleration[1]
                    imuMsg.linear_acceleration.z = -linear_acceleration[2]

                    pub[name].publish(imuMsg)

                    buttons = imu_manager.getButtonState(name)

                    pub[name+'_buttons'].publish(buttons)
            else:
                # Streaming mode
                if imu_manager.broadcast == False:
                    for name in imu_manager.imus:
                        # One message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        battery_voltage = Float64()
                        battery_percent = UInt8()
                        buttons = UInt8()

                        streaming_data = imu_manager.getStreamingData(name)
                        rospy.logdebug('Data: %s', streaming_data)
                        idx = 0

                        for slot in imu_manager.streaming_slots[name]:
                            if slot == 'getTaredOrientationAsQuaternion':
                                imuMsg.orientation.x = streaming_data[idx]
                                imuMsg.orientation.y = streaming_data[idx+1]
                                imuMsg.orientation.z = streaming_data[idx+2]
                                imuMsg.orientation.w = streaming_data[idx+3]
                                idx = idx+4

                            elif slot == 'getNormalizedGyroRate':
                                imuMsg.angular_velocity.x = streaming_data[idx]
                                imuMsg.angular_velocity.y = streaming_data[idx+1]
                                imuMsg.angular_velocity.z = streaming_data[idx+2]
                                idx = idx+3

                            elif slot == 'getCorrectedAccelerometerVector':
                                imuMsg.linear_acceleration.x = -streaming_data[idx]
                                imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                                imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                                idx = idx+3

                            elif slot == 'getBatteryVoltage':
                                battery_voltage = streaming_data[idx]
                                idx = idx+1

                            elif slot == 'getBatteryPercentRemaining':
                                battery_percent = streaming_data[idx].astype(numpy.uint8)
                                idx = idx+1

                            elif slot == 'getButtonState':
                                buttons = streaming_data[idx].astype(numpy.uint8)
                                idx = idx+1

                        # Publish streamed data
                        pub[name].publish(imuMsg)
                        pub[name+'_batteryVoltage'].publish(battery_voltage)
                        pub[name+'_batteryPercent'].publish(battery_percent)
                        pub[name+'_buttons'].publish(buttons)

                # Broadcast mode
                else:
                    for name in imu_manager.imus:
                        # One message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        buttons = UInt8()

                        streaming_data = imu_manager.devices[name].getStreamingBatch()
                        rospy.logdebug('Data: %s', streaming_data)
                        idx = 0

                        imuMsg.orientation.x = streaming_data[idx]
                        imuMsg.orientation.y = streaming_data[idx+1]
                        imuMsg.orientation.z = streaming_data[idx+2]
                        imuMsg.orientation.w = streaming_data[idx+3]
                        idx = idx+4

                        imuMsg.angular_velocity.x = streaming_data[idx]
                        imuMsg.angular_velocity.y = streaming_data[idx+1]
                        imuMsg.angular_velocity.z = streaming_data[idx+2]
                        idx = idx+3

                        imuMsg.linear_acceleration.x = -streaming_data[idx]
                        imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                        imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                        idx = idx+3

                        buttons = streaming_data[idx]

                        # Publish broadcast data
                        pub[name].publish(imuMsg)
                        pub[name+'_buttons'].publish(buttons)

        except TypeError:
            # print('TypeError occured!')
            pass

        # Wait for next loop
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
