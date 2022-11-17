import numpy as np
import time
import pandas as pd

num_arduinos = 1

def interp_nd(x, rx, ry):

    """ Interpolate new subdivision points given as (x) on a given spread of time (rx) and points (ry) """

    #breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data


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

def sensor_error_queue(error_queue, sensor_error_array, arduino_board_number):
    #global reduced_speed
    global arduino_output_list
    #arduino_board = arduino_output_list[arduino_board_number]
    sensor_error_array = np.transpose(sensor_error_array)
    error_queue = np.delete(error_queue, 0, 1)
    #Append error
    error_queue = np.hstack((error_queue, sensor_error_array))

    for i in range(2):
        err = error_queue[i]
        status = "OK"
        if min(err) > 0.3:
            status = "Warning"
            print(f"Warning: something is wrong near sensor {i}, board {arduino_board_number}")
            print(err)
            #print(f"point{point} refpoint {interp_ref_point}")
            #send_command("speed 20")
            reduced_speed = True
        elif reduced_speed:
            #send_command("speed 100")
            reduced_speed = False
        
        #arduino_output_list[arduino_board_number][i][0] = status
        #arduino_output_list[arduino_board_number][i][1] = err
    return error_queue

def main():
    error_queue = np.zeros((2, 5)) # 2 sensors
    global sensor_ref_array
    sensor_ref_array =read_sensor_reference("test_sensors")
    print(sensor_ref_array[0].shape)
    sample_point = np.transpose(np.array([250,250]))
    t_sample_start = 3.62
    interp_ref_point, interp_ref_time = interpolate_sensor_point(t_sample_start, 0)
    print(interp_ref_point)
    sensor_error_array = [(abs(distance - ref_distance)/ref_distance) for distance, ref_distance in zip(sample_point, interp_ref_point)]
    print(f"sensor error array{sensor_error_array}")
    #sensor_error_array = [np.array([1, 1])]
    error_queue = sensor_error_queue(error_queue, sensor_error_array, 0)
    print(error_queue)


if __name__ == "__main__":
    main()

