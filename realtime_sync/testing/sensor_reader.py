from threading import Thread
import serial
from serial.serialutil import *

# Najdi serial
ARDUINO_BOARD_PORT_ARRAY = ["/dev/tty.usbserial-1423130","/dev/tty.usbserial-1423140" ]
ARDUINO_BAUDRATE = 115200
num_arduinos = len(ARDUINO_BOARD_PORT_ARRAY)

def main():
    sensor_thread()

def sensor_thread():
    global collect_data
    collect_data = True

    # start as many threads as there are arduino boards, all checking errors independently
    for i in range(num_arduinos):
        thread = Thread(target=singleboard_datareader, args=[ARDUINO_BOARD_PORT_ARRAY[i], i])
        thread.start()

def singleboard_datareader(arduino_board_port, arduino_board_number):
    with open(f"alldata/sensor_data_{arduino_board_number}.csv", "a") as f:
        with serial.Serial() as ser:
            ser.baudrate = ARDUINO_BAUDRATE
            ser.port = arduino_board_port
            ser.open()
            txt = ser.readline()    # read line which just confirms that ranging is working
            txt = ser.readline()    # read line which just confirms that ranging is working
            while collect_data:   # sampling time
                for j in range(50):
                    for k in range(1):
                        ser.write(str.encode("\n"))
                        txt_array = ser.readline().decode("utf-8").strip()
                        txt_array = txt_array.split(",")
                        distance1 = txt_array[1]
                        distance2 = txt_array[4]
                        # Obdelava slabih meritev: naredi, kar želiš - trenutno samo vse da na 2000, zase še testiram
                        if txt_array[2] in ['2', '4', '7']:
                            if int(distance1) > 200 or int(distance1) < 200:
                                distance1 = 2000
                        if txt_array[5] in ['2', '4', '7']:
                            if int(distance2) > 200 or int(distance2) < 200:
                                distance2 = 2000
                        # Do stuff with data
                        f.write(f"{distance1},{distance2}\n")
                        print(f"{distance1},{distance2}\n")
                        

if __name__ == "__main__":
    main()