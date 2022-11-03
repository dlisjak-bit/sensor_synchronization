import time
import serial
from serial.serialutil import *
from queue import Queue
import numpy as np
from threading import Thread

updateFrequency = 125
samplingTime = 5   #sampling time in seconds
method = "euclidean"
reference_file = "reference_1664185538.csv"
thread_running = True
robot_speed = 100

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

NUCLEO_BOARD_PORT = "/dev/tty.usbmodem142303"

def parsePacket(txt):
    ID = int.from_bytes(txt[0:1], 'big', signed=False)
    temp = int.from_bytes(txt[1:2], 'big', signed=True)
    msmnts = np.zeros((3,16),np.uintc)
    for i in range(2,50, 3):
        #msmnts.append(int.from_bytes(txt[i:i+3], 'big', signed=True))
        detection = int((txt[i] >> 4) & 0x0f)
        status = int(txt[i] & 0x0f)
        distance = int.from_bytes(txt[i+1:i+3], 'big', signed=True)
        msmnts[0,int((i-2)/3)] = detection
        msmnts[1,int((i-2)/3)] = status
        msmnts[2,int((i-2)/3)] = distance 
    return ID, temp, msmnts
 

def new_sensor_reference():
    timestamp = str(int(time.time()))
    #sensor_filename = f"sensor_reference_{timestamp}.csv"
    sensor_filename = "test_sensors.csv"
    with open(sensor_filename, "w") as f:

        # Write header.
        f.write("ID,temp,t,")
        for i in range(16):
            f.write(f"detection{i},")
        for i in range(16):
            f.write(f"status{i},")
        for i in range(15):
            f.write(f"distance{i},")
        f.write("distance15\n")
        with serial.Serial() as ser:
            ser.baudrate = 460800
            ser.port = NUCLEO_BOARD_PORT
            ser.parity = PARITY_EVEN
            ser.stopbits = STOPBITS_ONE
            ser.open()

            ser.write(b'gs')     # start and stop ranging (bugfix)
            time.sleep(2)
            while ser.inWaiting(): ser.read()   # purge buffer
            
            ser.write(b'g')     # go
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)

            print("Sensor Reference Sampling started")
            for i in range(samplingTime):
                for j in range(45):
                    for k in range(8):
                        t_sample_start = 1
                        ID, temp, msmnts = parsePacket(ser.read(50))
                        line = f"{ID},{temp},{t_sample_start},"
                        detection = ""
                        status = ""
                        distance = ""
                        for i in msmnts[0]: 
                            detection += str(i)+","
                        for i in msmnts[1]:
                            status += str(i)+","
                        for i in msmnts[2]:
                            distance += str(i)+","
                        line += detection + status + distance + "\n"
                        f.write(line)
            ser.write(b's')     # stop

    return(sensor_filename)

new_sensor_reference()