import serial
from serial.serialutil import *
import numpy as np
from time import sleep
import datetime

NUCLEO_BOARD_PORT = "COM7"


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
    

if __name__ == "__main__":
    ranging_duration = 5   # ranging duration in seconds
    
    label = input("label:\t")
    #timestamp = str(int(time.time()))
    time = datetime.datetime.now()
    timestamp = f"{time.year}{time.month}{time.day}{time.hour}{time.minute}{time.second}" #microseconds, tzinfo
    filename = f"recordings\{timestamp}_{label}.csv"
    with open(filename, 'w') as f:
        with serial.Serial() as ser:
            ser.baudrate = 460800
            ser.port = NUCLEO_BOARD_PORT
            ser.parity = PARITY_EVEN
            ser.stopbits = STOPBITS_ONE
            ser.open()

            ser.write(b'gs')     # start and stop ranging (bugfix)
            sleep(2)
            while ser.inWaiting(): ser.read()   # purge buffer
            
            ser.write(b'g')     # go
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)
            
            for i in range(ranging_duration):   # sampling time
                t0 = datetime.datetime.now()
                
                for j in range(45):             # at 45 FPS
                    txts = []
                    for k in range(1):          # for each of the eight sensors
                        txts.append(ser.read(50))
                        #ser.read(50)
                    #f.write(f"{i}, {j};\n")
                        
                t1 = datetime.datetime.now()
                
                print("dt: ", t1-t0)
            ser.write(b's')     # stop
            print(txt)
            for t in txts:
                print(parsePacket(t))
            #f.write(txt.decode('utf-8'))
