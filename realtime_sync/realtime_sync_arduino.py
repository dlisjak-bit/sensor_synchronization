import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit
import os
import sys
import getopt
sys.path.append('..')
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from threading import Thread
import time, datetime
import serial
from serial.serialutil import *
from queue import Queue


# --------------GUIDE----------------------------------------
# before running test_2.urp, run initialize variables.py
# run the script as python3 realtime_sync_arduino.py <-t> <-r>
# optional flags: 
# -t: show calculated reference time in commandline (does not allow later user input)
# -r: record a reference loop before starting data collection
# Type user input: <command>
# stop: halts the process, must be restarted manually
# speed <value (1-100)>: percentage speed slider to be set
# exit: terminates program, robot continues operation


# system variables
updateFrequency = 125
samplingTime = 120   #sampling time in seconds
method = "euclidean"
reference_file = "reference_1664185538.csv"
thread_running = True
robot_speed = 100

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

ARDUINO_BOARD_PORT_ARRAY = ["/dev/tty.usbserial-14110", "/dev/tty.usbserial-14140"]
ARDUINO_BAUDRATE = 115200
num_arduinos = 2

argumentList = sys.argv[1:]

# Divide threads and start.
def main():
    
    global record_reference
    record_reference = False
    global show_time
    show_time = False
    record_reference, show_time = argparse(record_reference, show_time)
    
    t1 = Thread(target=data_processor_thread)
    if show_time:
        t2 = Thread(target=take_input_thread)

    t1.start()
    if show_time:
        t2.start()

def data_processor_thread():
    
    global reference_file
    global record_reference
    global show_time

    # Start robot
    connect_robot()
    if record_reference:
        robot_reference_file = new_robot_reference()
        print("Using new reference motion ...")

    tref,ref,pct100,do100 = read_robot_reference(robot_reference_file)
    robot_thread = Thread(target=robot_input_reader, args=[tref, ref])
    robot_thread.start()

    # Start sensors
    if record_reference:
        sensor_reference_file = new_sensor_reference()
        print("Recorded all sensor reference files.")

    global sensor_ref_array
    sensor_ref_array = read_sensor_reference(sensor_reference_file)
    t3 = Thread(target=sensor_thread)
    t3.start()

    print("Stopped receiving data.")

def argparse(record_reference, show_time):

    # Using arguments:
    options = "rt"
    # Long options
    long_options = ["reference", "time"]
    try:
        # Parsing argument
        arguments, values = getopt.getopt(argumentList, options, long_options)
        
        # checking each argument
        for currentArgument, currentValue in arguments:
    
            if currentArgument in ("-r", "--reference"):
                print ("Recording reference ...")
                record_reference = True
                
            elif currentArgument in ("-t", "--time"):
                show_time = True
                print ("Showing reference time ...")
                
    except getopt.error as err:
        # output error, and return with an error code
        print (str(err))
    
    return record_reference, show_time

# Math.
def normalize(v):
    v=np.array(v)
    norm=np.linalg.norm(v)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan

def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean

def interp_nd(x, rx, ry):

    """ Interpolate new subdivision points given as (x) on a given spread of time (rx) and points (ry) """

    #breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data

def get_normalized_t(ref, tref, pt, method="euclidean", spread=2, subdivisions=10):

    """ Use given method (euclidean/manhattan) to find nearest point in reference sample, interpolate nearby points to find precise reference time """

    if method=="euclidean":
        get_distance = get_euclidean
    elif method=="manhattan":
        get_distance = get_manhattan
    else:
        return -1
    d,p = get_distance(ref-pt)
    idxl = max(0, p[0][0]-spread)
    idxh = min(len(ref[0]), p[0][0]+spread+2)
    didx = idxh-idxl

    # Allocate array to divide time into more sample points
    samplepts = np.linspace(tref[idxl],tref[idxh-1], didx*subdivisions)

    # Interpolate the data between reference time points and fill samplepts array
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])

    # Find a more precise reference time estimate
    d,p = get_distance(subdivided-pt)
    return samplepts[p[0][0]]

# Robot.
def connect_robot():
    # connect to the robot
    global con
    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
    con.connect()

    # test connection (get controller version)
    global cv
    cv = con.get_controller_version()
    print(f"controller version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")

    # subscribe to the desired data
    global config 
    config = rtde_config.ConfigFile("cfg.xml")
    global state_names, state_types 
    state_names, state_types = config.get_recipe("state")
    con.send_output_setup(state_names, state_types, frequency = updateFrequency)

    # input bit for halting the process
    global input_65_names, input_65_types
    input_65_names, input_65_types = config.get_recipe("in65")
    global input_65
    input_65 = con.send_input_setup(input_65_names, input_65_types)

    # input bit for setting the speed
    global input_66_names, input_66_types
    input_66_names, input_66_types = config.get_recipe("in66")
    global input_66
    input_66 = con.send_input_setup(input_66_names, input_66_types)

    # input int for setting the speed
    global speed_int_names, speed_int_types
    speed_int_names, speed_int_types = config.get_recipe("speed_int")
    global speed_int
    speed_int = con.send_input_setup(speed_int_names, speed_int_types)

    if not con.send_start():
        print("failed to start data transfer")
        sys.exit()

    # setting default values so the program can run
    input_65.input_bit_register_65 = int(False)
    con.send(input_65)

    input_66.input_bit_register_66 = int(False)
    con.send(input_66)

    speed_int.input_int_register_25 = robot_speed
    con.send(speed_int)

def new_robot_reference():

    """ Record new reference motion. (Program needs to start by setting one digital output bit on and end by setting it off.
        Warning: end program by moving back to the beginning or set DO off after going back to start"""

    ta = []
    qa = []
    qda = []
    ssa = []
    tsfa = []
    doa = []

    timestamp = str(int(time.time()))
    ogfilename = f"reference_{timestamp}.csv"
    with open(ogfilename, "w") as f:
        f.write("t,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,ss,tsf,do\n") # write header row

        samplingState = "waiting for sync low"
        print("Reference Sampling started")
        for i in range(samplingTime):
            for j in range(updateFrequency):
                # receive the current state
                state = con.receive()
                
                if state is None:
                    print("connection lost, breaking")
                    break

                t = state.timestamp
                q = state.target_q
                qd = list(normalize(state.target_qd))
                ss = state.speed_scaling
                tsf = state.target_speed_fraction
                do = state.actual_digital_output_bits

                if samplingState == "waiting for sync low":
                    if (do%2)==0:
                        samplingState = "waiting for sync high"
                elif samplingState == "waiting for sync high":
                    if (do%2)==1:
                        print("cycle start detected")
                        samplingState = "collecting data"
                if samplingState == "collecting data":
                    if (do%2)==0:
                        samplingState = "finished"
                        break
                    ta.append(t)
                    qa.append(q)
                    qda.append(qd)
                    ssa.append(ss)
                    tsfa.append(tsf)
                    doa.append(do)
                    data = f"{t},{str(q)[1:-1]},{str(qd)[1:-1]},{ss},{tsf},{do}"
                    f.write(data+"\n")        
            print(f"Recorded {i+1} of {samplingTime} s")
            if samplingState == "finished":
                print("cycle complete - reference motion recorded")
                break

    if samplingState != "finished":
        print("WARNING: Program timed out before robot cycle was complete.")

    return (ogfilename)

def read_robot_reference(file):

    """ Read robot reference file. """

    data = []
    with open(file, 'r') as f:
        f.readline()    # throw away the header line
        for l in f:
            data.append([float(x) for x in l.strip().split(',')])   # read csv
    data = np.array(data).transpose()                               # transpose matrix
                                                                    # separate data:
    return data[0]-data[0][0], data[1:13], data[-2], data[-1]       # time points array ([time]-[start time]), 12 arrays (q0 do qd5) , array tsf, array do

def robot_input_reader(tref, ref):

    """ Continuously read data from robot and find reference time for each point """
    global normalized_t
    global keep_running
    keep_running = True
    print("Receiving data from robot.")
    while keep_running:
        for j in range(updateFrequency):

            # receive the current state
            state = con.receive()
        
            if state is None:
                print("connection lost, breaking")
                break
        
            datapt = np.array([])
            # if some past data points are required in the future, maybe import queue
            t = state.timestamp
            q = state.target_q
            qd = list(normalize(state.target_qd))
            ss = state.speed_scaling
            tsf = state.target_speed_fraction
            do = state.actual_digital_output_bits

            datapt = np.append(q,qd)
            datapt = np.array(datapt).transpose()
            datapt = datapt.reshape(12,1)
            normalized_t = get_normalized_t(ref, tref, datapt, method)
            if show_time:
                print(normalized_t, end="\r")

    con.send_pause()
    con.disconnect()

# User input
def take_input_thread():
    time.sleep(2)
    global keep_running
    keep_running = True
    while keep_running:
        user_input = input('Type user input: ')
        # doing something with the input
        send_command(user_input)

def send_command(user_input):

    global input_65
    global input_66
    split = user_input.split()
    
    if user_input == "stop" or user_input == "halt":
        input_65.input_bit_register_65 = int(True)
        con.send(input_65)
        time.sleep(0.5)
        input_65.input_bit_register_65 = int(False)
        con.send(input_65)
        print("Process halted.")
    elif split[0] == "speed":
        if (int(split[1]) < 101) & (int(split[1]) > 0):
            speed_int.input_int_register_25 = int(split[1])
            con.send(speed_int)
            input_65.input_bit_register_66 = int(True)
            con.send(input_65)
            time.sleep(0.5)
            input_65.input_bit_register_65 = int(False)
            con.send(input_65)
            time.sleep(1)
            print(f"Speed set to {split[1]}%")
        else: 
            print("Incorrect value. Speed must be between 1 and 100")
    elif user_input == "exit":
        print("Terminating program realtime_sync.py ... Robot will continue operation.")
        global keep_running
        keep_running=False
    else:
        print("Incorrect command.")

# Sensors
def sensor_thread():
    global collect_data
    collect_data = True

    # start as many threads as there are arduino boards, all checking errors independently
    for i in range(num_arduinos):
        thread = Thread(target=singleboard_datareader, args=[ARDUINO_BOARD_PORT_ARRAY[i], i])
        thread.start()

def new_sensor_reference():
    array = []
    for i in range(num_arduinos):
        array.append(reference_sensor_reader(ARDUINO_BOARD_PORT_ARRAY[i]))
    sensor_filename = "test_sensors"
    for i in range(2):
        with open(f"{sensor_filename}{i}.csv", 'w') as f:
            f.write(f"time,distance0,distance1\n")
            for line in array[i]:
                f.write(line)
    return(sensor_filename)

def read_sensor_reference(file):
    #read sensor reference and return matrix
    """ Read sensor reference file. """

    array = []
    for i in range(num_arduinos):
        data = []
        file_i = f"{file}{i}.csv"
        with open(file_i, 'r') as f:
            f.readline()    # throw away the header line
            for l in f:
                data.append([float(x) for x in l.strip().split(',')])   # read csv
        data = np.array(data).transpose()                               # transpose matrix
        array.append(data)
                                                                    # separate data:
    return array      # array[board][data]
   
def reference_sensor_reader(arduino_board_port):
    array = []
    with serial.Serial() as ser:
            ser.baudrate = ARDUINO_BAUDRATE
            ser.port = arduino_board_port
            #ser.parity = PARITY_EVEN
            #ser.stopbits = STOPBITS_ONE
            ser.open()
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)
            txt = ser.readline()    # read line which just confirms that ranging is working
            print(txt)
            samplingState = "waiting for sync low"              
            print(f"Sensor Reference Sampling started for board {arduino_board_port}")
            for i in range(samplingTime):
                for j in range(50):
                    state = con.receive()
                    do = state.actual_digital_output_bits

                    if state is None:
                        print("connection lost, breaking")
                        break

                    if samplingState == "waiting for sync low":
                        if (do%2)==0:
                            samplingState = "waiting for sync high"
                    elif samplingState == "waiting for sync high":
                        if (do%2)==1:
                            print("cycle start detected")
                            samplingState = "collecting data"
                    if samplingState == "collecting data":
                        if (do%2)==0:
                            samplingState = "finished"
                            break
                        t_sample = normalized_t.copy()
                        ser.write(str.encode("\n"))
                        txt_array = ser.readline().decode("utf-8").strip()
                        txt_array = txt_array.split(",")
                        line = f"{t_sample},{txt_array[1]},{txt_array[4]}\n"
                        array.append(line)
                        print(line)

                        #PROBLEM: prehitro pobiranje podatkov ga ubije
                        time.sleep(0.02)

                print(f"Time elapsed: {i}s")
                if samplingState == "finished":
                    print(f"cycle complete - reference for board {arduino_board_port} recorded")
                    break
            if samplingState != "finished":
                    print("WARNING: Program timed out before robot cycle was complete.")
    return array

def singleboard_datareader(arduino_board_port, arduino_board_number):
    queue_length = 5  
    error_queue = np.zeros((2, queue_length)) # 2 sensors

    with serial.Serial() as ser:
        ser.baudrate = ARDUINO_BAUDRATE
        ser.port = arduino_board_port
        ser.open()

        txt = ser.readline()    # read line which just confirms that ranging is working
        txt = ser.readline()    # read line which just confirms that ranging is working
        
        while collect_data:   # sampling time
            for j in range(50):
                for k in range(1):
                    t_sample_start = normalized_t.copy()
                    ser.write(str.encode("\n"))
                    txt_array = ser.readline().decode("utf-8").strip()
                    txt_array = txt_array.split(",")
                    point = np.transpose([float(txt_array[1]), float(txt_array[4])])
                    error_queue = check_sensors(point, t_sample_start, error_queue, arduino_board_number)

def sensor_error_queue(error_queue, sensor_error_array, arduino_board_number):
    print(error_queue.shape)
    print(error_queue)
    sensor_error_array = np.transpose(sensor_error_array)
    error_queue = np.delete(error_queue, 0, 1)
    #Append error
    error_queue = np.hstack((error_queue, sensor_error_array))

    for i in range(2):
        err = error_queue[i]
        if min(err) > 0.2:
            print(f"Warning: something is wrong near sensor {i}, board {arduino_board_number}")
    return error_queue

def check_sensors(point, t_sample_start, error_queue, arduino_board_number):
    #CHECK POINT DIMENSIONS - NEED ONLY DISTANCE
    interp_ref_point, interp_ref_time = interpolate_sensor_point(t_sample_start, arduino_board_number) # find reference point closest
    sensor_error_array = np.array([(abs(distance - ref_distance)/ref_distance) for distance, ref_distance in zip(point, interp_ref_point)]).transpose()
    error_queue = sensor_error_queue(error_queue, sensor_error_array, arduino_board_number)
    return error_queue

def interpolate_sensor_point(t_sample_start, arduino_board_number, spread=2, subdivisions=10):
    sensor_data = sensor_ref_array[arduino_board_number]
    sensor_tref = sensor_data[0]
    sensor_ref = sensor_data[1:3]

    t_dif = [abs(t_sample_start-point) for point in sensor_tref]
    idx = t_dif.index(min(t_dif))

    #sensor_ref = sensor_ref.transpose()

    idxl = max(0, idx - spread)
    idxh = min(len(sensor_tref)-1, idx + spread)
    didx = idxh-idxl

    samplepts = np.linspace(sensor_tref[idxl], sensor_tref[idxh-1], didx*subdivisions)
    subdivided = interp_nd(samplepts, sensor_tref[idxl:idxh], sensor_ref[:,idxl:idxh]) #test behaviour!

    t_dif = [abs(t_sample_start-t_point) for t_point in samplepts]
    idx = t_dif.index(min(t_dif))

    return subdivided[:, idx:idx+1], samplepts[idx]

if __name__ == "__main__":
     main()