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

ARDUINO_BOARD_PORT_ARRAY = ["/dev/tty.usbserial-14110", "/dev/tty.usbserial-14140"]

def new_sensor_reference(array0, array1):
    timestamp = str(int(time.time()))
    #sensor_filename = f"sensor_reference_{timestamp}.csv"
    sensor_filename = "test_sensors.csv"
    with open(sensor_filename, "w") as f:

        # Write header.
        f.write("t,")
        for i in range(4):
            f.write(f"status{i},")
        for i in range(3):
            f.write(f"distance{i},")
        f.write("distance3\n")
        

    return(sensor_filename)


def sensor_reader(ARDUINO_BOARD_PORT):
    array = []
    with serial.Serial() as ser:
            ser.baudrate = 115200
            ser.port = ARDUINO_BOARD_PORT
            #ser.parity = PARITY_EVEN
            #ser.stopbits = STOPBITS_ONE
            ser.open()
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)

            print("Sensor Reference Sampling started")
            for i in range(samplingTime):
                for j in range(50):
                    for k in range(1):
                        ser.write(str.encode("\n"))
                        txt_array = ser.readline().decode("utf-8").strip()
                        print(txt_array)
                        txt_array = txt_array.split(",")
                        line = f"{txt_array[1]},{txt_array[4]}\n"
                        array.append(line)
    return array
            

def sensor_divider():
    array = []
    array.append(sensor_reader(ARDUINO_BOARD_PORT_ARRAY[0]))
    array.append(sensor_reader(ARDUINO_BOARD_PORT_ARRAY[1]))
    sensor_filename = "test_sensors"
    for i in range(2):
        with open(f"{sensor_filename}{i}.csv", 'w') as f:
            f.write(f"distance0,distance1\n")
            for line in array[i]:
                f.write(line)

sensor_divider()

