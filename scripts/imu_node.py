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

"""

# # Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
import modules.imu as imu

# import ros msgs
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from ema_common_msgs.srv import SetUInt16

# import utilities
import yaml
import rospkg

def kill_node_callback(req):
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
    return {}

def set_imu_number_callback(req):
    imu_now = rospy.get_param('imu/wireless_id/pedal')
    msg = str(imu_now)
    if imu_now != req.data:
        if req.data > 0 and req.data <= 10:
            rospy.set_param('imu/wireless_id/pedal', req.data)
            rospack = rospkg.RosPack()
            imu_cfg_path = rospack.get_path('yostlabs_3space_imu')+'/config/imu.yaml'

            with open(imu_cfg_path, 'r') as f:
                imu_file = yaml.safe_load(f)
                imu_file['wireless_id']['pedal'] = req.data
            with open(imu_cfg_path, 'w') as f:
                yaml.safe_dump(imu_file, f, default_flow_style=False)

            msg = str(req.data)
            rospy.loginfo('Node shutdown: new imu number')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def main():
    # init imu node
    rospy.init_node('imu', anonymous=False)

    # get imu config
    imu_manager = imu.IMU(rospy.get_param('imu'))

    # list provided services
    services = {}
    services['kill_node'] = rospy.Service('imu/kill_node', Empty, kill_node_callback)
    services['set_imu_number'] = rospy.Service('imu/set_imu_number', SetUInt16, set_imu_number_callback)

    # list published topics
    pub = {}
    for name in imu_manager.imus:
        pub[name] = rospy.Publisher('imu/' + name, Imu, queue_size=10)
        # pub[name + '_buttons'] = rospy.Publisher('imu/' + name + '_buttons', Int8, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(200)

    # node loop
    while not rospy.is_shutdown():

        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            # POOLING MODE
            if imu_manager.streaming == False:
                ## messages are shared by all imus
                imuMsg = Imu()
                imuMsg.header.stamp = timestamp
                imuMsg.header.frame_id = frame_id
                buttons = Int8()
                
                for name in imu_manager.imus:
                    orientation = imu_manager.getQuaternion(name)

                    imuMsg.orientation.x = orientation[0]
                    imuMsg.orientation.y = orientation[1]
                    imuMsg.orientation.z = orientation[2]
                    imuMsg.orientation.w = orientation[3]

                    angular_velocity = imu_manager.getGyroData(name)

                    imuMsg.angular_velocity.x = angular_velocity[0]
                    imuMsg.angular_velocity.y = angular_velocity[1]
                    imuMsg.angular_velocity.z = angular_velocity[2]

                    linear_acceleration = imu_manager.getAccelData(name)

                    imuMsg.linear_acceleration.x = -linear_acceleration[0]
                    imuMsg.linear_acceleration.y = -linear_acceleration[1]
                    imuMsg.linear_acceleration.z = -linear_acceleration[2]

                    pub[name].publish(imuMsg)
                    
                    buttons = imu_manager.getButtonState(name)
                    
                    pub[name + '_buttons'].publish(buttons)
            else:
                # STREAMING MODE
                if imu_manager.broadcast == False:
                    for name in imu_manager.imus:
                        # one message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        # buttons = Int8()
                        
                        streaming_data = imu_manager.getStreamingData(name)
                        idx = 0
                        
                        for slot in imu_manager.streaming_slots[name]:
                            #print(name, slot)
                            
                            if slot == 'getTaredOrientationAsQuaternion':
                      
                                imuMsg.orientation.x = streaming_data[idx]
                                imuMsg.orientation.y = streaming_data[idx+1]
                                imuMsg.orientation.z = streaming_data[idx+2]
                                imuMsg.orientation.w = streaming_data[idx+3]
                                
                                idx = idx + 4
                                
                            elif slot == 'getNormalizedGyroRate':
                        
                                imuMsg.angular_velocity.x = streaming_data[idx]
                                imuMsg.angular_velocity.y = streaming_data[idx+1]
                                imuMsg.angular_velocity.z = streaming_data[idx+2]
                                
                                idx = idx + 3
                                
                            elif slot == 'getCorrectedAccelerometerVector':
                                
                                imuMsg.linear_acceleration.x = -streaming_data[idx]
                                imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                                imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                                
                                idx = idx + 3
                                
                            elif slot == 'getButtonState':

                                if type(streaming_data) == 'tuple':
                                    buttons = streaming_data[idx]
                                
                                    idx = idx + 1
                                else:
                                    # imu is only streaming button state, result is not a tuple
                                    buttons = streaming_data

                        # publish streamed data
                        pub[name].publish(imuMsg)
                        # pub[name + '_buttons'].publish(buttons)

                # BROADCAST MODE
                else:
                    for name in imu_manager.imus:
                        ## one message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        buttons = Int8()
                        
                        streaming_data = imu_manager.devices[name].getStreamingBatch()
                        
                        idx = 0
                        
                        imuMsg.orientation.x = streaming_data[idx]
                        imuMsg.orientation.y = streaming_data[idx+1]
                        imuMsg.orientation.z = streaming_data[idx+2]
                        imuMsg.orientation.w = streaming_data[idx+3]
                            
                        idx = idx + 4
                            
                        imuMsg.angular_velocity.x = streaming_data[idx]
                        imuMsg.angular_velocity.y = streaming_data[idx+1]
                        imuMsg.angular_velocity.z = streaming_data[idx+2]
                            
                        idx = idx + 3
                            
                        imuMsg.linear_acceleration.x = -streaming_data[idx]
                        imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                        imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                        
                        idx = idx + 3
                            
                        buttons = streaming_data[idx]

                        # publish streamed data
                        pub[name].publish(imuMsg)
                        # pub[name + '_buttons'].publish(buttons)

        except TypeError:
            print('TypeError occured!')

        # sleep until it's time to work again
        rate.sleep()
        
    # cleanup
    imu_manager.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
