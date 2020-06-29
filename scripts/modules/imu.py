"""

Particularly, this code is an auxiliary module for the IMU sensors
application. It consists of classes and methods that establish the serial
comm and give support in a deeper level.

The ROS node uses this code. It gives support in a deeper level, dealing
with minor details and is supposed to be independent of ROS, meaning it
shouldn't have to interact with ROS in any way. For example, it would
establish serial comm and treat raw measurements instead of publishing a
filtered sensor measurement as a ROS message to other ROS nodes.

"""

import time
import numpy
import serial
import struct
import binascii

class IMU(object):
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.broadcast = False
        self.streaming = False
        self.devices = {}
        self.dongles = []
        self.imus = []
        self.wired_imus = []
        self.wireless_imus = []
        self.sensor_list = []
        self.serialport = serial.Serial()

        for name in config_dict['dev_names']:
            dev_type = config_dict['dev_type'][name]

            if dev_type == 'DNG':
                wired_port = config_dict['wired_port'][name]

                self.serialport = serial.Serial(port=wired_port, baudrate=115200, timeout=0.001)
                time.sleep(0.1)
                self.serialport.flush()
                time.sleep(0.1)

                self.dongles.append(name)

            elif dev_type == 'WL' and config_dict['broadcast'] == False:
                imu_mode = config_dict['imu_mode'][name]
                self.imus.append(name)

                # if imu_mode == 'wired':
                #     wired_port = config_dict['wired_port'][name]
                #     self.devices[name] = ts_api.TSWLSensor(com_port=wired_port)
                #     self.wired_imus.append(name)

                if imu_mode == 'wireless':
                    wireless_dng = config_dict['wireless_dng'][name]
                    wireless_id = config_dict['wireless_id'][name]

                    # self.devices[name] = ts_api.TSWLSensor(logical_id=wireless_id, dongle=self.devices[wireless_dng])
                    self.wireless_imus.append(name)

        if config_dict['autocalibrate'] == True:
            self.autocalibrate()
            
        if config_dict['streaming'] == True:
            self.streaming = True
            
            self.streaming_interval = config_dict['streaming_interval']
            self.streaming_duration = config_dict['streaming_duration']
            self.streaming_delay = config_dict['streaming_delay']
            
            if config_dict['broadcast'] == False:
                self.streaming_slots = config_dict['streaming_slots']
                
                for name in self.imus:
                    if self.streaming_duration == 'unlimited':
                        self.streaming_duration = 0xFFFFFFFF
                        
                    wireless_id = config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
                    serial_port = self.serialport # Serial port of the respective dongle

                    # Set compass enable
                    msg = '>'+str(wireless_id)+',109,0'+'\n'
                    print(msg)
                    serial_port.write(msg)
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        out += '>> ' + serial_port.read(serial_port.inWaiting())
                    print(out)
                    out = ''

                    # Set streaming timing
                    msg = '>'+str(wireless_id)+',82'+str(self.streaming_interval)+\
                            str(self.streaming_duration)+str(self.streaming_delay)+'\n'
                    print(msg)
                    serial_port.write(msg)
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        out += '>> ' + serial_port.read(serial_port.inWaiting())
                    print(out)
                    out = ''

                    # Since this file was adapted for a specific application, the field "streaming_slots" from the configuration 
                    # file (imu.yalm) wont be treated here, but it should if a generalized application is desired

                    command = 0 # Code for GetTaredOrientationAsQuaternion, the only slot used here

                    # Set streaming slots
                    msg = '>'+str(wireless_id)+',80,'+str(command)+',255,255,255,255,255,255,255\n'
                    print(msg)
                    serial_port.write(msg)
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        out += '>> ' + serial_port.read(serial_port.inWaiting())
                    print(out)
                    out = ''

                    # Start streaming
                    serial_port.write('>'+str(wireless_id)+',85\n')
                    time.sleep(0.1)
                    while serial_port.inWaiting():
                        out = '>> ' + serial_port.read(serial_port.inWaiting())

                    print('Start')
                    
                    # # Start streaming
                    # # G: for some reason, startStreaming was a bad idea. without it, we get to 67Hz
                    # #self.devices[name].startStreaming()
            
            # else:
            #     self.broadcast = True
                
            #     for i in range(6): # Only checking the first six logical indexes
            #         sens = self.devices['pc'][i]
            #         if sens is not None:
            #             name = str(i)
            #             self.sensor_list.append(sens)
            #             self.devices[name] = sens
            #             self.imus.append(name)
                
            #     if self.streaming_duration == 'unlimited':
            #         self.streaming_duration = 0xFFFFFFFF
                    
            #     # Set IMU streams to the appropriate timing
            #     ts_api.global_broadcaster.setStreamingTiming(interval=self.streaming_interval,
            #                                                  duration=self.streaming_duration,
            #                                                  delay=self.streaming_delay,
            #                                                  delay_offset=12000,
            #                                                  filter=self.sensor_list)
                
            #     # Set IMU slots to getQuaternion, getGyroData, getAccelData and getButtonState
            #     ts_api.global_broadcaster.setStreamingSlots(slot0='getTaredOrientationAsQuaternion',
            #                                                 slot1='getNormalizedGyroRate',
            #                                                 slot2='getCorrectedAccelerometerVector',
            #                                                 slot3='getButtonState',
            #                                                 filter=self.sensor_list)
                
            #     # Start streaming
            #     ts_api.global_broadcaster.startStreaming(filter=self.sensor_list)

        # self.setRightHandedAxis()

########################################
# Calibration
########################################

    def calibrate(self, name): ## G: beginGyroscopeAutoCalibration, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle

        if dev_type == 'WL':
            msg = '>'+str(wireless_id)+',165\n'
            print(msg)
            serial_port.write(msg)
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                out += '>> ' + serial_port.read(serial_port.inWaiting())
            print(out)
            out = ''
            return 1

        else:
            print('calibrate not defined for dev_type = ', dev_type)
            return 0

########################################
# Set euler to YXZ
########################################

    def setEulerToYXZ(self, name): ## G: setEulerAngleDecompositionOrder with angle_order = 1, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle

        if dev_type == 'WL':
            msg = '>'+str(wireless_id)+',16,5\n'
            print(msg)
            serial_port.write(msg)
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                out += '>> ' + serial_port.read(serial_port.inWaiting())
            print(out)
            out = ''
            return 1

        else:
            print('setEulerToYXZ not defined for dev_type = ', dev_type)
            return 0

########################################
# Tare with current orientation
########################################

    def tare(self, name): ## G: tareWithCurrentOrientation, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle

        if dev_type == 'WL':
            msg = '>'+str(wireless_id)+',96\n'
            print(msg)
            serial_port.write(msg)
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                out += '>> ' + serial_port.read(serial_port.inWaiting())
            print(out)
            out = ''
            return 1

        else:
            print('tare not defined for dev_type = ', dev_type)
            return 0

########################################
# Check Buttons
########################################

    def getButtonState(self, name): ## G: getButtonState, works with TSWLSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            return self.devices[name].getButtonState()

        else:
            print('checkButtons not defined for dev_type = ', dev_type)
            return 0

########################################
# Get Quaternion
########################################

    def getQuaternion(self, name):
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            return self.devices[name].getTaredOrientationAsQuaternion()

        else:
            print('getQuaternion not defined for dev_type = ', dev_type)
            return 0

########################################
# Get Euler Angles
########################################

    def getEulerAngles(self, name): ## G: getTaredOrientationAsEulerAngles, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle
        angle = []

        if dev_type == 'WL':
            msg = '>'+str(wireless_id)+',1\n'
            print(msg)
            serial_port.write(msg)
            time.sleep(0.1)

            out = serial_port.inWaiting()
            if out > 0:
                data = bytearray(serial_port.read(out))

                # Each angle is a 4 byte float in the message. They are stored from byte 4 to 15. Pitch, Yaw, Roll
                temp = ''.join(chr(i) for i in data[4:8])
                pitch = struct.unpack('>f', temp)
                pitch = pitch[0]

                temp = ''.join(chr(i) for i in data[8:12])
                yaw = struct.unpack('>f', temp)
                yaw = yaw[0]

                temp = ''.join(chr(i) for i in data[12:16])
                roll = struct.unpack('>f', temp)
                roll = roll[0]         

                out = [pitch, yaw, roll]

                return out

        else:
            print('getEulerAngles not defined for dev_type = ', dev_type)
            return 0

########################################
# Get Gyro Data
########################################

    def getGyroData(self, name): ## G: getNormalizedGyroRate, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            return self.devices[name].getNormalizedGyroRate()

        else:
            print('getGyroData not defined for dev_type = ', dev_type)
            return 0

    def getAccelData(self, name): ## G: getCorrectedAccelerometerVector, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            return self.devices[name].getCorrectedAccelerometerVector()

        else:
            print('getAccelData not defined for dev_type = ', dev_type)
            return 0

    def getStreamingData(self, name): ## G: getStreamingBatch, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle

        if dev_type == 'WL':
            # The sensor might send more than one message at once so this piece of code is gonna handle that
            out = serial_port.inWaiting()
            if out > 0:
                data = serial_port.read(out)
                 # Decode to string and replace newline with space
                data = data.decode().replace('\r\n',' ')
                # Remove undesired characters

                # Create a list of strings separating each message if that's the case
                data = data.split(' ')
                data = list(filter(None, data))

                temp = data[-1] # Get latest message and ignore others
                temp = temp[3:] # Remove undesired first 3 bytes 
                temp = temp.split(',')
                temp = numpy.array(temp).astype(numpy.float)
                x = temp[0]
                y = temp[1]
                z = temp[2]
                w = temp[3]

                out = [x,y,z,w]

                return out

        else:
            print('getStreamingBatch not defined for dev_type = ', dev_type)
            return 0
    
    def shutdown(self):
        wireless_id = self.config_dict['wireless_id'][self.imus[0]] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle

        if self.streaming == True:
            for name in self.imus:
                print('shutting down')
            
                # Stop streaming
                msg = '>'+str(wireless_id)+',86\n'
                print(msg)
                serial_port.write(msg)
                time.sleep(0.1)

                serial_port.close()
                

    def autocalibrate(self):
        for name in self.imus:
            print("Calibrating", name)
            calibrationError = 10
            count = 0
            while calibrationError > 0.1 :
                count = count + 1
                ang = []
                while(len(ang) < 3):
                    self.setEulerToYXZ(name)
                    self.calibrate(name)
                    self.tare(name)
                    ang = self.getEulerAngles(name)
                oldError = calibrationError
                calibrationError = ang[0] + ang[1] + ang[2]

                if(oldError != calibrationError):
                    print("Error:", calibrationError)
                else:
                    print("Stopped calibrating",name,"due to error stabilization after",count,"attempts")
                    break
            print("Done")

    def setRightHandedAxis(self):
        # axes definitions
        # 0: X: R, Y: U, Z: F (left-handed system, standard operation)
        # 1: X: R, Y: F, Z: U (right-handed system)
        # 2: X: U, Y: R, Z: F (right-handed system)
        # 3: X: F, Y: R, Z: U (left-handed system)
        # 4: X: U, Y: F, Z: R (left-handed system)
        # 5: X: F, Y: U, Z: R (right-handed system)
        axes = 0
        x_inverted = 0
        y_inverted = 1
        z_inverted = 0

        axis_direction_byte = (x_inverted << 5) |(y_inverted << 4)  | (z_inverted << 3) | axes

        for name in self.imus:
            print("Changing", name, "to right handed axis")
            print("axis_direction_byte:", '{:08b}'.format(axis_direction_byte))
            self.devices[name].setAxisDirections(axis_direction_byte)

    def setLeftHandedAxis(self):
        # axes definitions
        # 0: X: R, Y: U, Z: F (left-handed system, standard operation)
        # 1: X: R, Y: F, Z: U (right-handed system)
        # 2: X: U, Y: R, Z: F (right-handed system)
        # 3: X: F, Y: R, Z: U (left-handed system)
        # 4: X: U, Y: F, Z: R (left-handed system)
        # 5: X: F, Y: U, Z: R (right-handed system)
        axes = 0
        x_inverted = 0
        y_inverted = 0
        z_inverted = 0

        axis_direction_byte = (x_inverted << 5) |(y_inverted << 4)  | (z_inverted << 3) | axes

        for name in self.imus:
            print("Changing", name, "to left handed axis")
            print("axis_direction_byte:", '{:08b}'.format(axis_direction_byte))
            self.devices[name].setAxisDirections(axis_direction_byte)

########################################
# Single Command
########################################

    def singleCommand(self, command): ## G: equivalent to writeRead, but using command bytes directly. delete?
        try:
            if self.serial_port is not None:
                self.serial_port.write(">" + str(self.address) + "," + command + "\n") # e escreve na porta
                dados = readData(self.serial_port)
                dados = dados.split(",")
                if int(dados[0]) == 0:
                    return dados
                else:
                    return "No answer"
            else:
                return 'Port error'

        except ValueError:
            return 'Error'
        return dados

def readData(port): ## G: yei api doesn't do separate reading, no worries. delete.
    dados = ''
    data = ''
    i = 1
    while dados == "":
        port.flush()
        data = port.read(port.inWaiting()) # le da porta bytearray
        dados = data.decode()  # transforma bytearray em string
        i += 1
        if i > 700:
            dados = 'No answer'
            break
    return dados
