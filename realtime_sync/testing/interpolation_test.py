import numpy as np

num_arduinos = 1

def main():
    reference0 = read_sensor_reference("test_sensors")
    #print(reference0)
    sensor_tref = reference0[0][0]
    #print(sensor_tref)
    sensor_ref = reference0[0][1:3]
    #print(sensor_ref)
    subdivisions = 10
    x = np.linspace(sensor_tref[0], sensor_tref[len(sensor_tref)-1], len(sensor_tref)*subdivisions)
    subdivided = interp_nd(x, sensor_tref[0:len(sensor_tref)-1], sensor_ref[:,0:len(sensor_tref)-1])
    new_reference = np.vstack((x, subdivided))
    print(new_reference)
    with open("interpolated_sensors.csv", "w") as f:
        for i in range(len(x)):
            f.write(f"{new_reference[0][i]},{int(new_reference[1][i])},{int(new_reference[2][i])}\n")

def interp_nd(x, rx, ry):

    """ Interpolate new subdivision points given as (x) on a given spread of time (rx) and points (ry) """

    #breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data


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

if __name__ == "__main__":
    main()