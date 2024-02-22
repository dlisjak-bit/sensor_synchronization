from threading import Thread
import serial
from serial.serialutil import *

# Najdi serial
ARDUINO_BOARD_PORT_ARRAY = ["/dev/tty.usbserial-1423110"]
ARDUINO_BAUDRATE = 115200
num_arduinos = len(ARDUINO_BOARD_PORT_ARRAY)


def main():
    sensor_thread()


def sensor_thread():
    global collect_data
    collect_data = True

    # start as many threads as there are arduino boards, all checking errors independently
    for i in range(num_arduinos):
        thread = Thread(
            target=singleboard_datareader, args=[ARDUINO_BOARD_PORT_ARRAY[i], i]
        )
        thread.start()


def singleboard_datareader(arduino_board_port, arduino_board_number):
    with open(f"alldata_attempt2/0test_data{arduino_board_number}.csv", "a") as f:
        with serial.Serial() as ser:
            ser.baudrate = ARDUINO_BAUDRATE
            ser.port = arduino_board_port
            ser.open()
            txt = (
                ser.readline()
            )  # read line which just confirms that ranging is working
            txt = (
                ser.readline()
            )  # read line which just confirms that ranging is working
            while collect_data:  # sampling time
                for j in range(50):
                    for k in range(1):
                        ser.write(str.encode("\n"))
                        txt_array = ser.readline().decode("utf-8").strip()
                        print(txt_array)
                        f.write(txt_array + "\n")


if __name__ == "__main__":
    main()
