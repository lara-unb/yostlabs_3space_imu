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
        self.manual_calibration = False
        self.tare_calibration = []
        self.baudrate = 115200 # default baudrate

        for name in config_dict['dev_names']:
            dev_type = config_dict['dev_type'][name]

            if dev_type == 'DNG':
                wired_port = config_dict['wired_port'][name]
                self.serialport = serial.Serial(port=wired_port, baudrate=self.baudrate, timeout=0.001)
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
            self.getAutocalibrationInfo()

        if config_dict['manual_calibration'] == True:
            for name in self.imus:
                # Set axis
                self.setEulerToYXZ(name)
                # Tare with fixed quartenion
                self.tareWithQuaternion(name)
            
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
                    serial_port.write(msg.encode())
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        temp_msg = serial_port.read(serial_port.inWaiting())
                        out += '>> ' + temp_msg.decode()
                    print(out)
                    out = ''

                    # Set streaming timing
                    msg = '>'+str(wireless_id)+',82'+str(self.streaming_interval)+\
                            str(self.streaming_duration)+str(self.streaming_delay)+'\n'
                    print(msg)
                    serial_port.write(msg.encode())
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        temp_msg = serial_port.read(serial_port.inWaiting())
                        out += '>> ' + temp_msg.decode()
                    print(out)
                    out = ''

                    # Since this file was adapted for a specific application, the field "streaming_slots" from the configuration 
                    # file (imu.yalm) wont be treated here, but it should if a generalized application is desired

                    command1 = 0 # Code for getTaredOrientationAsQuaternion
                    command2 = 33 # Code for getNormalizedGyroRate
                    command3 = 202 # Battery Percentage

                    # Set streaming slots
                    msg = '>'+str(wireless_id)+',80,'+str(command1)+\
                          ','+str(command2)+','+str(command3)+',255,255,255,255,255\n'
                    print(msg)
                    serial_port.write(msg.encode())
                    time.sleep(0.1)
                    out = ''
                    while serial_port.inWaiting():
                        temp_msg = serial_port.read(serial_port.inWaiting())
                        out += '>> ' + temp_msg.decode()
                    print(out)
                    out = ''

                    # Start streaming
                    msg = '>'+str(wireless_id)+',85\n'
                    serial_port.write(msg.encode())
                    time.sleep(0.1)
                    while serial_port.inWaiting():
                        temp_msg = serial_port.read(serial_port.inWaiting())
                        out = '>> ' + temp_msg.decode()
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
            serial_port.write(msg.encode())
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                temp_msg = serial_port.read(serial_port.inWaiting())
                out += '>> ' + temp_msg.decode('utf-8')
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
            serial_port.write(msg.encode())
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                temp_msg = serial_port.read(serial_port.inWaiting())
                out += '>> ' + temp_msg.decode()
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
            serial_port.write(msg.encode())
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                temp_msg = serial_port.read(serial_port.inWaiting())
                out += '>> ' + temp_msg.decode()
            print(out)
            out = ''
            return 1

        else:
            print('tare not defined for dev_type = ', dev_type)
            return 0

########################################
# Tare with quartenions
########################################

    def tareWithQuaternion(self, name):
        dev_type = self.config_dict['dev_type'][name]
        wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
        serial_port = self.serialport # Serial port of the respective dongle
        tare_quartenion = self.config_dict['tare_calibration']['imu'+str(wireless_id)] # Fixed quartenion

        if dev_type == 'WL':
            msg = '>'+str(wireless_id)+',97,'+','.join(map(str,tare_quartenion[0:4]))+'\n'
            print(msg)
            serial_port.write(msg.encode())
            time.sleep(0.1)
            out = ''
            while serial_port.inWaiting():
                temp_msg = serial_port.read(serial_port.inWaiting())
                out += '>> ' + temp_msg.decode()
            print(out)
            out = ''
            return 1
        
        else:
            print('tareWithQuartion not defined for dev_type = ', dev_type)
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
            serial_port.write(msg.encode())
            time.sleep(0.1)

            out = serial_port.inWaiting()
            if out > 0:
                data = bytearray(serial_port.read(out))

                # Each angle is a 4 byte float in the message. They are stored from byte 4 to 15. Pitch, Yaw, Roll
                temp = ''.join(chr(i) for i in data[4:8])
                pitch = struct.unpack('>f', temp.encode())
                pitch = pitch[0]

                temp = ''.join(chr(i) for i in data[8:12])
                yaw = struct.unpack('>f', temp.encode())
                yaw = yaw[0]

                temp = ''.join(chr(i) for i in data[12:16])
                roll = struct.unpack('>f', temp.encode())
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
                # print("DATA_RAW: ", data)

                # Decode to string and replace newline with space
                data = data.decode().replace('\r\n',' ')
                # print("DATA_DECODED: ", data)

                # Create a list of strings separating each message if that's the case
                data = data.split(' ')
                data = list(filter(None, data))
                # print("DATA_FILTERED: ", data)
                
                # Get the 2 latest messages and ignore others
                temp = data[-3] # Quartenios msg
                temp2 = data[-2] # Gyroscope msg
                temp3 = data[-1] # Battery level

                # Remove undesired first 3 bytes
                temp = temp[3:]  # only on quart
                # print("QUART_MSG: ", temp)
                # print("GYRO_MSG: ", temp2)
                
                temp = temp.split(',')
                temp2 = temp2.split(',')
                # print("QUART_MSG_SPLITTED: ", temp)
                # print("GYRO_MSG_SPLITTED: ", temp2)
                # print("BATTERY_MSG: ", temp3)

                # Convert to float
                temp = numpy.array(temp).astype(numpy.float)
                temp2 = numpy.array(temp2).astype(numpy.float)
                temp3 = numpy.array(temp3).astype(numpy.uint8)
                # print('QUART_MSG_CONVERTED:', temp)
                # print('GYRO_MSG_CONVERTED:', temp2)
                # print("BATTERY_MSG_CONVERTED: ", temp3)
                # Quartenions
                x = temp[0]
                y = temp[1]
                z = temp[2]
                w = temp[3]
                # qt_msg = [x, y, z, w]
                # print('QUART_PUB_MSG: ', qt_msg)

                # Gyroscope
                v1 = temp2[0]
                v2 = temp2[1]
                v3 = temp2[2]
                # v_msg = [v1, v2, v3]
                # print('GYRO_PUB_MSG: ', v_msg)

                # Battery
                b = temp3

                out = [x,y,z,w,v1,v2,v3,b]
                # out = 0
                return out
            else:
                return None

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
                serial_port.write(msg.encode())
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

    def getAutocalibrationInfo(self):
        for name in self.imus:
            dev_type = self.config_dict['dev_type'][name]
            wireless_id = self.config_dict['wireless_id'][name] # Logical id of WL device in associated dongle's wireless table
            serial_port = self.serialport # Serial port of the respective dongle

            if dev_type == 'WL':
                # Get current tare in quartenions
                msg = '>'+str(wireless_id)+',128\n'
                print(msg)
                serial_port.write(msg.encode())
                time.sleep(0.1)
                out = ''
                while serial_port.inWaiting():
                    temp_msg = serial_port.read(serial_port.inWaiting())
                    out += '>> ' + temp_msg.decode()
                print('tareAsQuartenion: ', out)
                out = ''

                # Get compass calibration coeff - matrix
                msg = '>'+str(wireless_id)+',162\n'
                print(msg)
                serial_port.write(msg.encode())
                time.sleep(0.1)
                out = ''
                while serial_port.inWaiting():
                    temp_msg = serial_port.read(serial_port.inWaiting())
                    out += '>> ' + temp_msg.decode()
                print('compass_coeffs: ', out)
                out = ''

                # Get gyroscope calibration coeff - matrix
                msg = '>'+str(wireless_id)+',163\n'
                print(msg)
                serial_port.write(msg.encode())
                time.sleep(0.1)
                out = ''
                while serial_port.inWaiting():
                    temp_msg = serial_port.read(serial_port.inWaiting())
                    out += '>> ' + temp_msg.decode()
                print('gyro_coeffs: ', out)
                out = ''
                return 1

            else:
                print('getAutocalibrationInfo not defined for dev_type = ', dev_type)
                return 0

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
