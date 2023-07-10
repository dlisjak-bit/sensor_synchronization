import matplotlib.pyplot as plt
import numpy as np

# from perlin_noise import PerlinNoise
# import timeit
# import os
import sys
import getopt
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from threading import Thread
import serial

# from serial.serialutil import *

# from queue import Queue
from rich.console import Console
from rich.table import Table
from matplotlib.animation import FuncAnimation

sys.path.append("..")

# TODO: ne adaptira nazaj na visok speed
# TODO: TCP ignorira ob dvigovanju
# TODO: adaptiranje za vsak senzor posebej

# --------------GUIDE----------------------------------------
# before running test_2.urp, run initialize variables.py
# run the script as python3 realtime_sync_arduino.py <-t> <-r>
# optional flags:
# -t: show calculated reference time in commandline (does not allow later user
# input)
# -r: record a reference loop before starting data collection
# Type user input: <command>
# stop: halts the process, must be restarted manually
# speed <value (1-100)>: percentage speed slider to be set
# exit: terminates program, robot continues operation


# system variables
updateFrequency = 125
samplingTime = 120  # sampling time in seconds
method = "euclidean"
robot_reference_file = "robot_reference.csv"
sensor_reference_file = "sensor_ref"
thread_running = True
robot_speed = 100
sensor_data_filename = "alldata/sensor_data_"
interp_sensor_data_filename = "alldata/interpolated_sensors"
error_logs_filename = "alldata/error_logs"
adapted_reference_filename = "alldata/adapted_reference"

SAFETY_DISTANCE = 1000

ROBOT_HOST = "192.168.65.244"  # actual robot
# ROBOT_HOST = '192.168.56.102'   # virtual robot
ROBOT_PORT = 30004

ARDUINO_BOARD_PORT_ARRAY = [
    "/dev/tty.usbserial-1423110",
    "/dev/tty.usbserial-1423120",
    "/dev/tty.usbserial-1423130",
    "/dev/tty.usbserial-1423140",
]
ARDUINO_BAUDRATE = 115200
num_arduinos = len(ARDUINO_BOARD_PORT_ARRAY)
NUM_REF_CYCLES = 1
collision = False
sensors_active = False
REDUCED_SPEED_PERCENTAGE = 0

argumentList = sys.argv[1:]

# check for reduced speed: if no error, safely put speed back to normal
reduced_speed = []
inactive_sensors = []
for i in range(num_arduinos):
    reduced_speed.append([False, False])
    inactive_sensors.append([False, False])

# for each joint, enter sensor coordinates as tuple ex. (3,1)
JOINT_SENSOR_MAP = [
    [],
    [],
    [],
    [],
    [],
    [],
]

# Clear out files
for i in range(4):
    with open(f"alldata/adapted_reference{i}.csv", "w") as f:
        f.close()
    with open(f"alldata/error_logs{i}.csv", "w") as f:
        f.close()
    with open(f"alldata/interpolated_sensors{i}.csv", "w") as f:
        f.close()
    with open(f"alldata/sensor_data{i}.csv", "w") as f:
        f.close()

# Create table element for all threads
# (Robot), num_arduinos * (arduino_board), (input?)
robot_output = ["", ""]
arduino_output_list = []
for i in range(num_arduinos):
    arduino_output_list.append([[[""], [""]], [[""], [""]]])
last_command = ""


# Plotting logs
T_MAX = 0
error_display = []
graph_output_list = []
speed_display = []
for i in range(num_arduinos):
    graph_output_list.append([])
    error_display.append([])
    speed_display.append([])
    for j in range(3):
        graph_output_list[i].append([])
        error_display[i].append([])
    for j in range(2):
        speed_display[i].append([])
sensor_measurement_display = np.array([])


# Divide threads and start.
def main():
    global record_reference
    record_reference = False
    global show_time
    show_time = False
    record_reference, show_time = argparse(record_reference, show_time)

    # Start data processors and input thread
    t1 = Thread(target=data_processor_thread)
    if show_time:
        t2 = Thread(target=take_input_thread)

    t1.start()
    if show_time:
        t2.start()

    # Wait so we get actual data:
    time.sleep(30)
    # Matplotlib graphs must be in main thread:
    # graph_output()


def data_processor_thread():
    global robot_reference_file
    global sensor_reference_file
    global record_reference
    global show_time
    global T_MAX
    global sensor_ref_array
    global sensors_active

    # Start robot
    connect_robot()
    if record_reference:
        send_command("speed 100")
        robot_reference_file = new_robot_reference()
        print("Using new reference motion ...")
    tref, ref, forces, pct100, do100 = read_robot_reference(
        robot_reference_file
    )
    # print(forces)
    T_MAX = max(tref)
    interp_forces = interpolate_forces(tref, forces)
    # All interpolation of new sensor reference arrays will go to T_MAX
    robot_thread = Thread(
        target=robot_input_reader, args=[tref, ref, interp_forces]
    )
    robot_thread.start()

    # Start sensors
    if record_reference:
        # send_command("speed 20")
        sensor_reference_file = new_sensor_reference()
        send_command("speed 100")
        print("Recorded all sensor reference files.")
    sensor_ref_array = read_sensor_reference(sensor_reference_file)

    # Interpolate each arduino's sensor reference to 1000Hz
    new_ref_file = sensor_ref_interp(sensor_ref_array)
    sensor_ref_array = read_sensor_reference(new_ref_file)
    print("Interpolated new sensor reference")

    # Start output thread
    t0 = Thread(target=output_thread)
    t0.start()

    # Start Sensors
    t3 = Thread(target=sensor_thread)
    t3.start()

    sensors_active = True
    # Start taking input - needs work if its at the same time as output
    # t2 = Thread(target=take_input_thread)
    # t2.start()

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
                print("Recording reference ...")
                record_reference = True

            elif currentArgument in ("-t", "--time"):
                show_time = True
                print("Showing reference time ...")

    except getopt.error as err:
        # output error, and return with an error code
        print(str(err))

    return record_reference, show_time


def sensor_ref_interp(sensor_ref_array):
    filename = "alldata/interpolated_sensors"
    subdivisions = 1000
    t_max_rounded = int(T_MAX * subdivisions) / subdivisions
    x = np.arange(0, t_max_rounded * subdivisions) * 0.001
    for i in range(num_arduinos):
        sensor_tref = sensor_ref_array[i][0]
        sensor_ref = sensor_ref_array[i][1:3]
        subdivided = interp_nd(
            x,
            sensor_tref[0 : len(sensor_tref)],
            sensor_ref[:, 0 : len(sensor_tref)],
        )
        new_reference = np.vstack((x, subdivided))
        # Also write to file - not necessary, but easy to just read again
        with open(f"{filename}{i}.csv", "w") as f:
            f.write("time,distance0,distance1\n")
            for i in range(1, len(x)):
                f.write(
                    f"{int(new_reference[0][i]*subdivisions)/subdivisions},{int(new_reference[1][i])},{int(new_reference[2][i])}\n"
                )
    return filename


# Math.
def normalize(v):
    v = np.array(v)
    norm = np.linalg.norm(v)
    if norm == 0:
        norm = np.finfo(v.dtype).eps
    return v / norm


def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan


def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean


def interp_nd(x, rx, ry):
    """
    Interpolate new subdivision points given as (x) on a given spread of time
    (rx) and points (ry).
    """

    # breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        # print(x, rx, ry[i])
        try:
            data[i] = np.interp(x, rx, ry[i])
        except ValueError as err:
            print(x, rx, ry[i])
            print(err)
    return data


def get_normalized_t(
    ref, tref, pt, previous_t, method="euclidean", spread=2, subdivisions=10
):
    """Use given method (euclidean/manhattan) to find nearest point in reference sample, interpolate nearby points to find precise reference time"""

    if method == "euclidean":
        get_distance = get_euclidean
    elif method == "manhattan":
        get_distance = get_manhattan
    else:
        return -1
    # Search only ref with max 0.1s ahead of previous_t
    tref_np = np.asarray(tref)
    t_end = previous_t + 3
    t_start = previous_t
    idx_end = min((np.abs(tref_np - t_end)).argmin(), len(tref))
    idx_start = (np.abs(tref_np - t_start)).argmin()
    ref_interp = ref[:, idx_start : idx_end + 1]
    d, p = get_distance(ref_interp - pt)
    idxl = max(0, p[0][0] + idx_start)
    idxh = min(len(ref[0]), p[0][0] + idx_start + spread + 2)
    didx = idxh - idxl

    # Allocate array to divide time into more sample points
    samplepts = np.linspace(tref[idxl], tref[idxh - 1], didx * subdivisions)

    # Interpolate the data between reference time points and fill samplepts array
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:, idxl:idxh])

    # Find a more precise reference time estimate
    d, p = get_distance(subdivided - pt)
    return samplepts[p[0][0]]


def interpolate_forces(tref, forces):
    subdivisions = 1000
    t_max_rounded = int(T_MAX * subdivisions) / subdivisions
    x = np.arange(0, t_max_rounded * subdivisions) * 0.001
    subdivided = interp_nd(
        x,
        tref[0 : len(tref)],
        forces[:, 0 : len(tref)],
    )
    new_forces = subdivided
    return new_forces


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
    con.send_output_setup(state_names, state_types, frequency=updateFrequency)

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

    # input bit for acknowledging cycle start
    global input_67_names, input_67_types
    input_67_names, input_67_types = config.get_recipe("in67")
    global input_67
    input_67 = con.send_input_setup(input_67_names, input_67_types)

    # input int for setting the speed
    global speed_int_names, speed_int_types
    speed_int_names, speed_int_types = config.get_recipe("speed_int")
    global speed_int
    speed_int = con.send_input_setup(speed_int_names, speed_int_types)

    if not con.send_start():
        print("failed to start data transfer")
        sys.exit()

    # setting default values so the program can run
    input_67.input_bit_register_67 = int(False)
    con.send(input_67)

    input_65.input_bit_register_65 = int(False)
    con.send(input_65)

    input_66.input_bit_register_66 = int(False)
    con.send(input_66)

    speed_int.input_int_register_25 = robot_speed
    con.send(speed_int)


def new_robot_reference():
    """Record new reference motion. (Program needs to start by setting one digital output bit on and end by setting it off.
    Warning: end program by moving back to the beginning or set DO off after going back to start
    """

    global robot_output
    global input_67

    ta = []
    qa = []
    qda = []
    ssa = []
    tsfa = []
    doa = []
    forces = []

    # timestamp = str(int(time.time()))
    ogfilename = "robot_reference.csv"
    with open(ogfilename, "w") as f:
        f.write(
            "t,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,ss,f0,f1,f2,f3,f4,f5,tsf,do\n"
        )  # write header row

        samplingState = "waiting for sync low"
        robot_output[0] = samplingState
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
                force = list(state.ft_raw_wrench)

                if samplingState == "waiting for sync low":
                    if (do % 2) == 0:
                        input_67.input_bit_register_67 = int(True)
                        con.send(input_67)
                        samplingState = "waiting for sync high"
                        print(samplingState)
                        robot_output[0] = samplingState
                elif samplingState == "waiting for sync high":
                    if (do % 2) == 1:
                        print("cycle start detected")
                        samplingState = "collecting data"
                        robot_output[0] = samplingState
                if samplingState == "collecting data":
                    if (do % 2) == 0:
                        input_67.input_bit_register_67 = int(True)
                        con.send(input_67)
                        samplingState = "finished"
                        break
                    input_67.input_bit_register_67 = int(False)
                    con.send(input_67)
                    ta.append(t)
                    qa.append(q)
                    qda.append(qd)
                    ssa.append(ss)
                    tsfa.append(tsf)
                    doa.append(do)
                    forces.append(force)
                    data = f"{t},{str(q)[1:-1]},{str(qd)[1:-1]},{ss},{str(force)[1:-1]},{tsf},{do}"
                    f.write(data + "\n")
                    robot_output[0] = samplingState
            print(f"Recorded {i+1} of {samplingTime} s")
            if samplingState == "finished":
                print("cycle complete - reference motion recorded")
                break

    if samplingState != "finished":
        print("WARNING: Program timed out before robot cycle was complete.")

    return ogfilename


def read_robot_reference(file):
    """Read robot reference file."""

    data = []
    with open(file, "r") as f:
        f.readline()  # throw away the header line
        for line in f:
            data.append(
                [float(x) for x in line.strip().split(",")]
            )  # read csv
    data = np.array(data).transpose()  # transpose matrix
    # separate data:
    return (
        data[0] - data[0][0],
        data[1:13],
        data[14:20],
        data[-2],
        data[-1],
    )  # time points array ([time]-[start time]), 12 arrays (q0 do qd5) , array tsf, array do


def robot_input_reader(tref, ref, interp_forces):
    """
    Continuously read data from robot and find reference time for each point.
    """
    global normalized_t
    global keep_running
    global robot_output
    global collision
    global recording_sensor_reference
    collision_time = 0.0
    keep_running = True
    previous_t = 0.0
    print("Receiving data from robot.")
    samplingState = "waiting for sync low"
    while keep_running:
        for j in range(updateFrequency):
            # receive the current state
            state = con.receive()
            datapt = np.array([])
            # if some past data points are required in the future, maybe import queue
            t = state.timestamp
            q = state.target_q
            qd = list(normalize(state.target_qd))
            ss = state.speed_scaling
            tsf = state.target_speed_fraction
            do = state.actual_digital_output_bits

            if state is None:
                print("connection lost, breaking")
                break
            if samplingState == "waiting for sync low":
                if (do % 2) == 0:
                    # if not recording_sensor_reference:
                    #     input_67.input_bit_register_67 = int(True)
                    #     con.send(input_67)
                    samplingState = "waiting for sync high"
                    robot_output[0] = samplingState
            elif samplingState == "waiting for sync high":
                if (do % 2) == 1:
                    previous_t = 0.0

                    samplingState = "collecting data"
                    robot_output[0] = samplingState
            if samplingState == "collecting data":
                if (do % 2) == 0:
                    samplingState = "waiting for sync high"
                    # if not recording_sensor_reference:
                    #     input_67.input_bit_register_67 = int(True)
                    #     con.send(input_67)
                    # END OF CYCLE MARKER
                    for i in range(num_arduinos):
                        with open(f"{error_logs_filename}{i}.csv", "a+") as f:
                            f.write("-\n")
                        with open(f"{sensor_data_filename}{i}.csv", "a+") as f:
                            f.write("-\n")
            # if not recording_sensor_reference:
            #     input_67.input_bit_register_67 = int(False)
            #     con.send(input_67)
            datapt = np.append(q, qd)
            datapt = np.array(datapt).transpose()
            datapt = datapt.reshape(12, 1)
            normalized_t = get_normalized_t(
                ref, tref, datapt, previous_t, method
            )
            # Collision detection. Only adaptation can set collision back to
            # False:

            target_c = np.array(state.target_current).astype(float)
            actual_c = np.array(state.actual_current).astype(float)
            if sensors_active:
                if check_forces(
                    target_c, actual_c, normalized_t, np.array(qd)
                ):
                    if not collision:
                        print("COLLISION DETECTED !!!!!!!!!!!!!!!!")
                    collision = True
                    collision_time = normalized_t.copy()
            if (
                collision
                and normalized_t - collision_time > 0.5
                or normalized_t - collision_time < -0.2
            ):
                collision = False
            # if normalized_t < previous_t:
            #     print(f"ERR, time {normalized_t} prev {previous_t}\n")
            check_active_sensors(qd)
            previous_t = normalized_t
            robot_output[0] = "Monitoring time"
            robot_output[1] = f"{normalized_t} s"
            if show_time:
                print(normalized_t, end="\r")

    con.send_pause()
    con.disconnect()


def check_forces(
    interp_forces, rec_forces, normalized_t, target_joint_velocities
):
    dif = interp_forces - rec_forces
    sgn = -np.sign(target_joint_velocities)
    # če sta predznaka nasprotna, tega ne želimo
    if np.any(sgn * dif > 1.6):
        print("force in opposite direction")
    elif np.any(abs(dif) > 1.6):
        print("force in same direction")
    return np.any(sgn * dif > 1.65)


def check_active_sensors(target_joint_velocities):
    """
    Map joints to sensors and check if they should be active.
    (Moving away from object: not relevant - disable sensor).
    """
    global inactive_sensors
    # Map joints to sensors and c
    for idx, sensor_list in enumerate(JOINT_SENSOR_MAP):
        for sensor in sensor_list:
            # print(f"Analysing sensor {sensor}:")
            # print(target_joint_velocities[idx])
            if (
                target_joint_velocities[idx] < 0
                and not inactive_sensors[sensor[0]][sensor[1]]
            ):
                inactive_sensors[sensor[0]][sensor[1]] = True
                # print("set to inactive")
            elif (
                target_joint_velocities[idx] > 0
                and inactive_sensors[sensor[0]][sensor[1]]
            ):
                # print("set to active")
                inactive_sensors[sensor[0]][sensor[1]] = False


# User input and terminal output
def take_input_thread():
    global user_input
    user_input = ""
    time.sleep(2)
    global keep_running
    keep_running = True
    while keep_running:
        user_input = input("Type user input: ")
        # doing something with the input
        send_command(user_input)


def send_command(user_input):
    global input_65
    global input_66
    global last_command
    last_command = user_input
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
            input_66.input_bit_register_66 = int(True)
            # input_66.input_bit_register_66 = int(False)
            con.send(input_66)
            # time.sleep(1)
            # print(f"Speed set to {split[1]}%")
            # print(split[1])
            time.sleep(0.1)
            input_66.input_bit_register_66 = int(False)
            con.send(input_66)
        else:
            print("Incorrect value. Speed must be between 1 and 100")
    elif user_input == "exit":
        print(
            "Terminating program realtime_sync.py ... Robot will continue operation."
        )
        global keep_running
        keep_running = False
    else:
        print("Incorrect command.")


def output_thread():
    global user_input
    user_input = ""
    while False:
        table = Table(title="Monitoring output")
        table.add_column("Source")
        table.add_column("Board no.")
        table.add_column("Status")
        table.add_column("Data")

        table.add_row("Last command", last_command)
        table.add_row("Robot", "", robot_output[0], robot_output[1])
        for idx, arduino_board in enumerate(arduino_output_list):
            for sensor_idx, sensor in enumerate(arduino_board):
                table.add_row(
                    f"Sensor{sensor_idx}",
                    f"{idx}",
                    f"{sensor[0]}",
                    f"{sensor[1]}",
                )

        console = Console()
        console.print(table, end="\r")
        time.sleep(0.2)


def graph_output():
    # Must be started from the main thread.
    global fig, axs
    fig, axs = plt.subplots(num_arduinos, 2)
    ani = FuncAnimation(fig, graph_updater, interval=1000)
    plt.show()


def graph_updater(i):
    global fig, axs
    for num_arduino in range(num_arduinos):
        for num_sensor in range(2):
            ax = axs[num_arduino][num_sensor]
            # Update data
            ax.cla()
            # Plot: sensor data, time
            ax.plot(
                graph_output_list[num_arduino][num_sensor],
                graph_output_list[num_arduino][0],
            )
            warning = "No warning"
            ax.text(
                warning.format(graph_output_list[num_arduino][num_sensor][-1])
            )


# Sensors
def sensor_thread():
    global collect_data
    collect_data = True

    # start as many threads as there are arduino boards, all checking errors independently
    for i in range(num_arduinos):
        thread = Thread(
            target=singleboard_datareader,
            args=[ARDUINO_BOARD_PORT_ARRAY[i], i],
        )
        thread.start()


def new_sensor_reference():
    global sensor_ref_array_to_write
    global recording_sensor_reference
    sensor_ref_array_to_write = []
    input_67.input_bit_register_67 = int(False)
    con.send(input_67)
    # Get state for all sensors from one feed
    recording_sensor_reference = True
    get_state = Thread(target=reference_sensor_robotcom)
    get_state.start()
    # Read from sensors
    for i in range(num_arduinos):
        sensor_ref_array_to_write.append([])
    sensor_threads = []
    for i in range(num_arduinos):
        t = Thread(
            target=reference_sensor_reader,
            args=[ARDUINO_BOARD_PORT_ARRAY[i], i],
        )
        sensor_threads.append(t)
    for t in sensor_threads:
        t.start()
    for t in sensor_threads:
        t.join()
    # Stop getting state
    recording_sensor_reference = False
    sensor_filename = "sensor_ref"
    for i in range(num_arduinos):
        with open(f"{sensor_filename}{i}.csv", "w") as f:
            f.write("time,distance0,distance1\n")
            for line in sensor_ref_array_to_write[i]:
                f.write(line)
    return sensor_filename


def read_sensor_reference(file):
    # read sensor reference and return matrix
    """Read sensor reference file."""

    array = []
    for i in range(num_arduinos):
        data = []
        file_i = f"{file}{i}.csv"
        with open(file_i, "r") as f:
            f.readline()  # throw away the header line
            for line in f:
                data.append(
                    [float(x) for x in line.strip().split(",")]
                )  # read csv
        data = np.array(data).transpose()  # transpose matrix
        array.append(data)
        # separate data:
    return array  # array[board][data]


def reference_sensor_robotcom():
    global recording_sensor_reference
    global sensor_ref_state
    while recording_sensor_reference:
        sensor_ref_state = con.receive()
    return


def reference_sensor_reader(arduino_board_port, arduino_board_index):
    """Read reference sensor array"""
    global sensor_ref_array_to_write
    global arduino_output_list
    global sensor_ref_state
    array = []
    with serial.Serial() as ser:
        ser.baudrate = ARDUINO_BAUDRATE
        ser.port = arduino_board_port
        # ser.parity = PARITY_EVEN
        # ser.stopbits = STOPBITS_ONE
        ser.open()
        txt = (
            ser.readline()
        )  # read line which just confirms that ranging is working
        print(txt)
        txt = (
            ser.readline()
        )  # read line which just confirms that ranging is working
        print(txt)
        samplingState = "waiting for sync low"
        status = "waiting for sync low"
        arduino_output_list[arduino_board_index][0][0] = status
        arduino_output_list[arduino_board_index][1][0] = status
        print(
            f"Sensor Reference Sampling started for board {arduino_board_port}"
        )
        num_cycles_done = 0
        for i in range(samplingTime):
            for j in range(50):
                time.sleep(0.01)
                state = sensor_ref_state
                do = state.actual_digital_output_bits

                if state is None:
                    # print("connection lost, breaking")
                    status = "connection lost, breaking"
                    arduino_output_list[arduino_board_index][0][0] = status
                    arduino_output_list[arduino_board_index][1][0] = status
                    break

                if samplingState == "waiting for sync low":
                    if (do % 2) == 0:
                        input_67.input_bit_register_67 = int(True)
                        con.send(input_67)
                        samplingState = "waiting for sync high"
                        status = samplingState
                        arduino_output_list[arduino_board_index][0][0] = status
                        arduino_output_list[arduino_board_index][1][0] = status
                elif samplingState == "waiting for sync high":
                    if (do % 2) == 1:
                        print("cycle start detected")
                        samplingState = "collecting data"
                        status = samplingState
                        arduino_output_list[arduino_board_index][0][0] = status
                        arduino_output_list[arduino_board_index][1][0] = status
                if samplingState == "collecting data":
                    if (do % 2) == 0:
                        # if num_cycles_done < NUM_REF_CYCLES - 1:
                        # num_cycles_done += 1
                        # else:
                        input_67.input_bit_register_67 = int(True)
                        con.send(input_67)
                        samplingState = "finished"
                        break
                    input_67.input_bit_register_67 = int(False)
                    con.send(input_67)
                    t_sample = normalized_t.copy()
                    ser.write(str.encode("\n"))
                    txt_array = ser.readline().decode("utf-8").strip()
                    txt_array = txt_array.split(",")
                    try:
                        distance0 = int(txt_array[1])
                    except IndexError as err:
                        print(err, txt_array)
                        print(arduino_board_port + "ERROR")
                    distance1 = int(txt_array[4])
                    # Handle weird signals
                    if txt_array[2] in ["2", "4", "7"]:
                        # Distance failure
                        print(
                            f"Error code {txt_array[2]}, sensor 0{arduino_board_index}, {distance0}"
                        )
                        distance0 = SAFETY_DISTANCE + 1

                    if txt_array[5] in ["2", "4", "7"]:
                        # Distance failure
                        distance1 = SAFETY_DISTANCE + 1
                        print(
                            f"Error code {txt_array[5]}, sensor 0{arduino_board_index}, {distance1}"
                        )
                    if distance0 > SAFETY_DISTANCE:
                        distance0 = SAFETY_DISTANCE
                    if distance1 > SAFETY_DISTANCE:
                        distance1 = SAFETY_DISTANCE
                    line = f"{t_sample},{distance0},{distance1}\n"
                    array.append(line)
                    status = samplingState
                    arduino_output_list[arduino_board_index][0][0] = status
                    arduino_output_list[arduino_board_index][1][0] = status
                    # print(line)
                    arduino_output_list[arduino_board_index][0][1] = line
                    arduino_output_list[arduino_board_index][1][1] = line
                    # PROBLEM: prehitro pobiranje podatkov ga ubije
                    time.sleep(0.005)

            print(f"Time elapsed: {i}s")
            if samplingState == "finished":
                print(
                    f"cycle complete - reference for board {arduino_board_port} recorded"
                )
                break
        if samplingState != "finished":
            print(
                "WARNING: Program timed out before robot cycle was complete."
            )
    # Sort array by time
    sensor_ref_array_to_write[arduino_board_index] = array


def singleboard_datareader(arduino_board_port, arduino_board_number):
    """Read and check data from one board with two sensors"""
    global graph_output_list
    global collision
    queue_length = 5
    error_queue = np.zeros((2, queue_length))  # 2 sensors
    ref_time = []
    ref_distance0 = []
    ref_distance1 = []
    # collision = False
    with open(f"alldata/sensor_data_{arduino_board_number}.csv", "a") as f:
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
            start_time = normalized_t.copy()
            while collect_data:  # sampling time
                for j in range(50):
                    for k in range(1):
                        t_sample_start = normalized_t.copy()
                        ser.write(str.encode("\n"))
                        txt_array = ser.readline().decode("utf-8").strip()
                        txt_array = txt_array.split(",")
                        distance0 = int(txt_array[1])
                        distance1 = int(txt_array[4])
                        # if txt_array[2] == "2":
                        #     print(
                        #         f"STATUS CODE 2, BOARD{arduino_board_number}S0, DISTANCE {distance0}\n"
                        #     )
                        # if txt_array[5] == "2":
                        #     # print(
                        #     #     f"STATUS CODE 2, BOARD{arduino_board_number}S1, DISTANCE {distance1}\n"
                        #     # )
                        if txt_array[2] in ["2", "4", "7"]:
                            # Distance failure
                            distance0 = SAFETY_DISTANCE + 1
                        if txt_array[5] in ["2", "4", "7"]:
                            # Distance failure
                            distance1 = SAFETY_DISTANCE + 1
                        if distance0 > SAFETY_DISTANCE + 1:
                            distance0 = SAFETY_DISTANCE
                        if distance1 > SAFETY_DISTANCE + 1:
                            distance1 = SAFETY_DISTANCE
                        point = np.transpose(
                            [float(distance0), float(distance1)]
                        )
                        # Append to file for graph output
                        graph_output_list[arduino_board_number][0].append(
                            t_sample_start
                        )
                        graph_output_list[arduino_board_number][1].append(
                            float(distance0)
                        )
                        graph_output_list[arduino_board_number][2].append(
                            float(distance1)
                        )
                        # For adapting reference
                        ref_time.append(t_sample_start)
                        ref_distance0.append(float(distance0))
                        ref_distance1.append(float(distance1))
                        # Write to file
                        f.write(
                            f"{t_sample_start},{distance0},{distance1},{REDUCED_SPEED_PERCENTAGE}\n"
                        )
                        # Check for errors
                        error_queue = check_sensors(
                            point,
                            t_sample_start,
                            error_queue,
                            arduino_board_number,
                        )
                        if t_sample_start - start_time > 0.3:
                            # 2 normalized seconds have gone by: adapt reference
                            start_time = t_sample_start
                            current_ref_array = np.vstack(
                                (ref_time, ref_distance0, ref_distance1)
                            )
                            # print("adapting")
                            if not collision:
                                thread_adaptor = Thread(
                                    target=adapt_reference,
                                    args=[
                                        current_ref_array,
                                        arduino_board_number,
                                    ],
                                )
                                thread_adaptor.start()
                                # print("adapting")
                                # adapt_reference(current_ref_array, arduino_board_number)
                            else:
                                print("Not adapting due to collision")
                            ref_time = []
                            ref_distance0 = []
                            ref_distance1 = []
                        if t_sample_start - start_time < -0.1:
                            # We entered a new cycle - remove measurements from new cycle and adapt those from previous
                            # print("NEW CYCLE ADAPT")
                            start_time = t_sample_start
                            ref_time = ref_time[
                                0 : np.where(ref_time == max(ref_time))[0][0]
                                + 1
                            ]
                            ref_distance0 = ref_distance0[
                                0 : np.where(ref_time == max(ref_time))[0][0]
                                + 1
                            ]
                            ref_distance1 = ref_distance1[
                                0 : np.where(ref_time == max(ref_time))[0][0]
                                + 1
                            ]
                            current_ref_array = np.vstack(
                                (ref_time, ref_distance0, ref_distance1)
                            )
                            if not collision:
                                thread_adaptor = Thread(
                                    target=adapt_reference,
                                    args=[
                                        current_ref_array,
                                        arduino_board_number,
                                    ],
                                )
                                thread_adaptor.start()
                            ref_time = []
                            ref_distance0 = []
                            ref_distance1 = []
                            # Also empty out graph_output
                            for i in range(3):
                                graph_output_list[arduino_board_number][i] = []


def adapt_reference(current_ref_array, arduino_board_number):
    """
    Adapt the reference sensor point array with appropriate weights on a
    given time interval if no collision occured
    """
    global sensor_ref_array
    global error_display
    w_new = 0.5
    w_old = 1 - w_new
    arr0 = list(current_ref_array[0])  # time 0
    arr1 = list(current_ref_array[1])  # Readings 0
    arr2 = list(current_ref_array[0])
    arr3 = list(current_ref_array[2])  # readings 1
    subdivisions = 1000
    t_start = int(current_ref_array[0][0] * subdivisions) / subdivisions
    t_end = int(current_ref_array[0][-1] * subdivisions) / subdivisions
    # Remove measurements with error codes from adaptation?
    for idx, reading in enumerate(arr1):
        if reading == 1001:
            arr1.pop(idx)
            arr0.pop(idx)
    for idx, reading in enumerate(arr2):
        if reading == 1001:
            arr3.pop(idx)
            arr2.pop(idx)
    del current_ref_array
    current_ref_array = [
        arr0,
        arr1,
        arr2,
        arr3,
    ]
    for i in [0, 2]:
        if len(current_ref_array[i + 1]) <= 1:
            continue
        if len(current_ref_array[i]) <= 1:
            continue
        sensor_number = 0 if i == 0 else 1
        subdivisions = 1000
        # Round end and start times
        # t_start = int(current_ref_array[i][0] * subdivisions) / subdivisions
        # t_end = int(current_ref_array[i][-1] * subdivisions) / subdivisions
        # Find index of where to do weighted average
        start_index = max(int(t_start * subdivisions) - 1, 0)
        end_index = min(
            int(t_end * subdivisions) - 1, int(T_MAX * subdivisions - 2)
        )
        # Interpolate recorded reference
        x = np.arange(0, t_end * subdivisions) * 0.001 + t_start
        x = []
        for j in range(int(t_end * subdivisions - t_start * subdivisions)):
            x.append(j * 1 / subdivisions + t_start)
        x = np.array(x)
        ref_with_zeroes = np.array(
            [current_ref_array[i + 1], np.zeros(len(current_ref_array[i + 1]))]
        )
        # sensor_tref_rec = refe
        subdivided = np.interp(
            x,
            current_ref_array[i],
            current_ref_array[i + 1],
        )
        for j in range(start_index, end_index - 1):
            old_weighted = (
                w_old
                * sensor_ref_array[arduino_board_number][sensor_number][j]
            )
            new_weighted = w_new * subdivided[j - start_index]
            sensor_ref_array[arduino_board_number][sensor_number][j] = (
                old_weighted + new_weighted
            )

    # NEW VERSION --------^^^^^

    # if len(current_ref_array[0]) == 0:
    #     print("Adapting empty array (error codes)")
    #     return
    # sensor_tref_rec = current_ref_array[0]
    # sensor_ref_rec = current_ref_array[1:3]

    # subdivisions = 1000

    # # Round end and start times
    # t_start = int(sensor_tref_rec[0] * subdivisions) / subdivisions
    # t_end = int(sensor_tref_rec[-1] * subdivisions) / subdivisions

    # # Find index of where to do weighted average
    # start_index = max(int(t_start * subdivisions) - 1, 0)
    # end_index = min(
    #     int(t_end * subdivisions) - 1, int(T_MAX * subdivisions - 2)
    # )
    # # print(f"start index:{start_index}, end index: {end_index}")

    # # Interpolate recorded reference
    # x = np.arange(0, t_end * subdivisions) * 0.001 + t_start
    # x = []
    # for i in range(int(t_end * subdivisions - t_start * subdivisions)):
    #     x.append(i * 1 / subdivisions + t_start)
    # x = np.array(x)
    # subdivided = interp_nd(
    #     x,
    #     sensor_tref_rec[0 : len(sensor_tref_rec)],
    #     sensor_ref_rec[:, 0 : len(sensor_tref_rec)],
    # )

    # with open(
    #     f"alldata/adapted_reference{arduino_board_number}.csv", "a"
    # ) as f:
    #     for i in range(start_index, end_index - 1):
    #         for j in range(1, 3):
    #             old_weighted = (
    #                 w_old * sensor_ref_array[arduino_board_number][j][i]
    #             )
    #             new_weighted = w_new * subdivided[j - 1][i - start_index]
    #             sensor_ref_array[arduino_board_number][j][i] = (
    #                 old_weighted + new_weighted
    #             )
    #         f.write(
    #             f"{sensor_ref_array[arduino_board_number][0][i]},{sensor_ref_array[arduino_board_number][1][i]},{sensor_ref_array[arduino_board_number][2][i]}\n"
    #         )

    for num_arduino in range(num_arduinos):
        with open(f"alldata/error_logs{num_arduino}.csv", "a") as f:
            for i in range(len(error_display[num_arduino][0])):
                f.write(
                    f"{error_display[num_arduino][0][i]},{error_display[num_arduino][1][i]},{error_display[num_arduino][2][i]},{REDUCED_SPEED_PERCENTAGE}\n"
                )
    for i in range(num_arduinos):
        for j in range(3):
            error_display[i][j].clear()
    # print(f"Reference adapted at board {arduino_board_number}.")
    return


def check_sensors(point, t_sample_start, error_queue, arduino_board_number):
    """
    Compare expected point with actual point - call sensor_error_queue to
    analyse error
    """
    # CHECK POINT DIMENSIONS - NEED ONLY DISTANCE
    global error_display
    interp_ref_point, interp_ref_time = interpolate_sensor_point(
        t_sample_start, arduino_board_number
    )  # find reference point closest
    sensor_error_array = np.array([[0.0], [0.0]])
    error_display[arduino_board_number][0].append(t_sample_start)
    if all(ref_distance > 10 for ref_distance in interp_ref_point) or True:
        for i in range(2):
            distance = point[i]
            ref_distance = interp_ref_point[i]
            error = (ref_distance - distance) / (ref_distance + 0.0001)
            sensor_error_array[i][0] = error
            error_display[arduino_board_number][i + 1].append(error)
        # sensor_error_array = np.transpose(np.array([[(abs(distance -
        # ref_distance)/ref_distance)] for distance, ref_distance in
        # zip(point, interp_ref_point)]))
        # print(sensor_error_array)
    error_queue = sensor_error_queue(
        error_queue,
        sensor_error_array,
        arduino_board_number,
        point,
        interp_ref_point,
    )
    return error_queue


def calculate_error_threshold(distance):
    reduced_percentage = 80 - 80 / 1000 * distance
    if 60 < reduced_percentage <= 80:
        return 80
    elif 40 < reduced_percentage <= 60:
        return 60
    elif 20 < reduced_percentage <= 40:
        return 40
    return 20


def sensor_error_queue(
    error_queue,
    sensor_error_array,
    arduino_board_number,
    point,
    interp_ref_point,
):
    """Create error queue for sensor measurements. Slow down robot if necessary."""
    # print(error_queue.shape)
    # print(error_queue)
    global reduced_speed
    global arduino_output_list
    global REDUCED_SPEED_PERCENTAGE
    # sensor_error_array = np.transpose(sensor_error_array)
    error_queue = np.delete(error_queue, 0, 1)
    # Append error
    error_queue = np.hstack((error_queue, sensor_error_array))
    # print(reduced_speed[arduino_board_number][0],reduced_speed[arduino_board_number][1])
    for i in range(2):
        err = error_queue[i]
        status = "OK"
        if min(err) > 0.3:
            status = "Warning"
            # print(err)
            # print(f"point{point} refpoint {interp_ref_point}")
            reduced_percentage = calculate_error_threshold(point[i])
            if (
                not reduced_speed[arduino_board_number][i]
                and not inactive_sensors[arduino_board_number][i]
            ) or (
                reduced_percentage > REDUCED_SPEED_PERCENTAGE
                and not inactive_sensors[arduino_board_number][i]
            ):
                reduced_percentage = max(
                    reduced_percentage, REDUCED_SPEED_PERCENTAGE
                )
                REDUCED_SPEED_PERCENTAGE = reduced_percentage
                speed_reduce_thread = Thread(
                    target=send_command,
                    args=[f"speed {100 - REDUCED_SPEED_PERCENTAGE}"],
                )
                # print(inactive_sensors[3][1])
                speed_reduce_thread.start()
                reduced_speed[arduino_board_number][i] = True
                print(
                    f"Warning: something is wrong near sensor {i}, board {arduino_board_number}"
                )
                print(point, interp_ref_point, min(err))
        elif (
            max(err) < 0.3 and reduced_speed[arduino_board_number][i]
        ) or inactive_sensors[arduino_board_number][i]:
            # print(reduced_speed)
            reduced_speed[arduino_board_number][i] = False
            reduced_speed_overall = False
            for arduino_board in reduced_speed:
                for sensor in arduino_board:
                    if sensor:
                        reduced_speed_overall = True
            if not reduced_speed_overall:
                speed_increase_thread = Thread(
                    target=send_command, args=["speed 100"]
                )
                speed_increase_thread.start()
                REDUCED_SPEED_PERCENTAGE = 0
        # print(reduced_speed)
        arduino_output_list[arduino_board_number][i][0] = status
        arduino_output_list[arduino_board_number][i][1] = err
    return error_queue


def interpolate_sensor_point(
    t_sample_start, arduino_board_number, spread=2, subdivisions=10
):
    """Interpolate expected sensor point at certain time.
    Currently works by interpolating in advance and only finding
    wanted sensor_point through array that has 1000 subdivisions per second"""
    subdivisions = 1000
    t_rounded = int(t_sample_start * subdivisions) / subdivisions
    index = min(
        int(t_rounded * subdivisions) - 1,
        len(sensor_ref_array[arduino_board_number][1]) - 1,
    )
    return (
        np.transpose(
            [
                sensor_ref_array[arduino_board_number][1][index],
                sensor_ref_array[arduino_board_number][2][index],
            ]
        ),
        sensor_ref_array[arduino_board_number][0][index],
    )

    while False:
        sensor_data = sensor_ref_array[arduino_board_number]
        sensor_tref = sensor_data[0]
        sensor_ref = sensor_data[1:3]

        t_dif = [abs(t_sample_start - point) for point in sensor_tref]
        idx = t_dif.index(min(t_dif))

        # sensor_ref = sensor_ref.transpose()

        idxl = max(0, idx - spread)
        idxh = min(len(sensor_tref) - 1, idx + spread)
        didx = idxh - idxl

        samplepts = np.linspace(
            sensor_tref[idxl], sensor_tref[idxh - 1], didx * subdivisions
        )
        subdivided = interp_nd(
            samplepts, sensor_tref[idxl:idxh], sensor_ref[:, idxl:idxh]
        )  # test behaviour!

        t_dif = [abs(t_sample_start - t_point) for t_point in samplepts]
        idx = t_dif.index(min(t_dif))

    return subdivided[:, idx : idx + 1], samplepts[idx]


# Output graphs and text


if __name__ == "__main__":
    main()
