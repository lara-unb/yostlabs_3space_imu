import serial
import time
import binascii

number_of_sensors = 1
p = False
time_to_measure = 10
command = 0

addresses = [7,8,2,3,4,5,1]

portIMU = '/dev/tty.usbmodem1411' # rPi
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

def read_sensors(portIMU):
    counters = [0] * len(addresses)
    t0 = time.time()

    while 1:
        if time.time()-t0 >= time_to_measure:

            for i in range(len(addresses)):
                print('f'+str(i+1)+' = ' + str(counters[i] / time_to_measure))
            break

        bytes_to_read = serial_port.inWaiting()
        if bytes_to_read > 0:
            data = serial_port.read(bytes_to_read)
            id_str = binascii.hexlify(data[1].encode('utf-8'))
            if p:
                print(id_str)

            try:
                id = int(id_str)
                for i in range(len(addresses)):
                    if id == addresses[i]:
                        counters[i] += 1
            except ValueError:
                print('NAN')

    for i in range(len(addresses)):
        serial_port.write('>'+str(addresses[i])+',86\n')


    serial_port.close()

read_sensors(portIMU)