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
import serial
import struct

class IMU(object):
    # Add more using Yostlabs user manual and legacy lib threespace_api for original names
    # Method: 'command_name': (command_number, data_lengh in bytes, number of messages (default 1))
    command_dict = {
    # Orientation Commands
    'getTaredOrientationAsQuaternion': (0, 4, 1),
    'getTaredOrientationAsEulerAngles': (1, 3, 1),
    'getTaredOrientationAsRotationMatrix': (2, 9, 1),
    'getTaredOrientationAsAxisAngle': (3, 4, 1),
    'getTaredOrientationAsTwoVector': (4, 6, 1),
    'getDifferenceQuaternion': (5, 4, 1),
    'getUntaredOrientationAsQuaternion': (6, 4, 1),
    'getUntaredOrientationAsEulerAngles': (7, 3, 1),
    'getUntaredOrientationAsRotationMatrix': (8, 9, 1),
    'getUntaredOrientationAsAxisAngle': (9, 4, 1),
    'getUntaredOrientationAsTwoVector': (10, 6, 1),
    'getTaredTwoVectorInSensorFrame': (11, 6, 1),
    'getUntaredTwoVectorInSensorFrame': (12, 6, 1),
    # Normalized Data Commands
    'getAllNormalizedComponentSensorData': (32, 9, 1),
    'getNormalizedGyroRate': (33, 3, 1),
    'getNormalizedAccelerometerVector': (34, 3, 1),
    'getNormalizedCompassVector': (35, 3, 1),
    # Corrected Data Commands
    'getAllCorrectedComponentSensorData': (37, 9, 1),
    'getCorrectedGyroRate': (38, 3, 1),
    'getCorrectedAccelerometerVector': (39, 3, 1),
    'getCorrectedCompassVector': (40, 3, 1),
    'getCorrectedLinearAccelerationInGlobalSpace': (41, 3, 1),
    'getCorrectedRawGyroData': (48, 3, 1),
    'getCorrectedRawAccelerometerData': (49, 3, 1),
    'getCorrectedRawCompassData': (50, 3, 1),
    # Raw Data Commands
    'getAllRawComponentSensorData': (64, 9, 1),
    'getRawGyroscopeRate': (65, 3, 1),
    'getRawAccelerometerData': (66, 3, 1),
    'getRawCompassData': (67, 3, 1),
    # Battery Commands
    'getBatteryVoltage': (201, 1, 1),
    'getBatteryPercentRemaining': (202, 1, 1),
    # HID Commands
    'getButtonState': (250, 1, 1),
    'null': (255, 0, 0),
    }

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
        self.slot_number = {}
        self.msgs_number = {}

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

                    ## Generalized application

                    # Get IMU slots according to config file
                    padded_slots = list(self.streaming_slots[name])
                    
                    # Add null if there is less than 9 commands
                    for i in range(0,8):
                        try:
                            padded_slots[i]
                        except IndexError:
                            padded_slots.append('null')

                    # Get each slot number from first dict column
                    self.slot_number[name] = []
                    for slot in padded_slots:
                        cmd_number = self.command_dict[slot][0]
                        self.slot_number[name].append(cmd_number)

                    # Calculate total msgs in streamed data (used in getStreamData)
                    self.msgs_number[name] = sum(self.command_dict[slot][1] for slot in padded_slots)

                    msg = '>'+str(wireless_id)+',80,'+','.join(map(str,self.slot_number[name][0:8]))+'\n'
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

                    # Start streaming
                    # G: for some reason, startStreaming was a bad idea. without it, we get to 67Hz
                    # self.devices[name].startStreaming()
            
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
        total_msgs = self.msgs_number[name] # Total msgs in streaming data
        if dev_type == 'WL':
            bytes_waiting = serial_port.inWaiting()
            # There is a new msg of bytes_waiting size
            if bytes_waiting > 0:
                data_buffer = serial_port.read(bytes_waiting)  # Get the msg as bytes
                data_buffer = data_buffer.decode()  # Unicode string
                # print('\n\n\nStreaming Data Raw:\n'+data_buffer)
                # Get at least one batch in case the sensor sends more than one
                while data_buffer:
                    try:
                        # Parse the 3 byte header
                        fail_byte, logical_id, msg_len = (ord(x) for x in data_buffer[0:3])
                        if fail_byte:  # Communication failure
                            print('Error: non zero fail byte')
                            return 0
                        data_buffer = data_buffer[3:]
                        # Check if the msg is from the requested sensor
                        if logical_id == wireless_id:
                            mixed_data = data_buffer[:msg_len]
                            # Convert the str msg into list and cleanup
                            data_list = [x for x in mixed_data.split('\r\n') if x != '']
                            # print('Valid Data List:\n'+str(data_list))
                            data = []
                            for measurement in data_list:
                                # Convert to numeric values
                                try:  # Try int first
                                    values = [int(x) for x in measurement.split(',')]
                                    data += values
                                    continue
                                # If necessary try float after
                                except (TypeError, ValueError):
                                    pass
                                values = [float(x) for x in measurement.split(',')]
                                data += values
                            # print('\nData Final:\n'+str(data))
                            return data if len(data) == total_msgs else 0
                        data_buffer = data_buffer[msg_len:]  # Try next batch
                    except (IndexError, TypeError, ValueError) as e:
                        print(e)
                        break
            return 0
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
