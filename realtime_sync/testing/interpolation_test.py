import numpy as np

num_arduinos = 1


def main():

    # Interpoliramo referenco
    reference0 = read_sensor_reference("test_sensors")
    #print(reference0)
    sensor_tref = reference0[0][0]
    #print(sensor_tref)
    sensor_ref = reference0[0][1:3]
    #print(sensor_ref)
    subdivisions = 1000
    t_max = 7.31989749 # Recimo. 
    t_max = int(t_max*subdivisions)/subdivisions
    print(t_max)
    x = np.linspace(0, t_max, int(t_max*subdivisions)) # find t_max
    subdivided = interp_nd(x, sensor_tref[0:len(sensor_tref)-1], sensor_ref[:,0:len(sensor_tref)-1])
    new_reference = np.vstack((x, subdivided))
    print(new_reference)
    with open("interpolated_sensors1.csv", "w") as f:
        for i in range(len(x)):
            f.write(f"{int(new_reference[0][i]*subdivisions)/subdivisions},{int(new_reference[1][i])},{int(new_reference[2][i])}\n")
    
    # Pridejo novi podatki
    print(sensor_ref.shape)
    sensor_tref_novi = np.empty(len(sensor_tref)-5)
    for idx, time in enumerate(sensor_tref_novi):
        sensor_tref_novi[idx] = sensor_tref[idx] + 0.0005
    sensor_ref_novi = np.empty((2, len(sensor_ref[0])-5))
    for idx, meritev in enumerate(sensor_ref_novi[0]):
        nova_meritev = sensor_ref[0][idx] -5
        sensor_ref_novi[0][idx] = nova_meritev
    for idx, meritev in enumerate(sensor_ref_novi[1]):
        nova_meritev = sensor_ref[1][idx] -5
        sensor_ref_novi[1][idx] = nova_meritev
    with open("nova_meritev.csv", "w") as f:
        for i in range(len(sensor_tref_novi)):
            f.write(f"{sensor_tref_novi[i]},{sensor_ref_novi[0][i]},{sensor_ref_novi[1][i]}\n")

    subdivided = interp_nd(x, sensor_tref_novi[0:len(sensor_tref_novi)-1], sensor_ref_novi[:,0:len(sensor_tref_novi)-1])
    recorded_reference = np.vstack((x, subdivided))

    with open("interpolated_sensors2.csv", "w") as f:
        for i in range(1, len(x)):
            f.write(f"{int(recorded_reference[0][i]*subdivisions)/subdivisions},{int(recorded_reference[1][i])},{int(recorded_reference[2][i])}\n")

    # Pol vzamemo weighted povprecje in naredimo:
    with open("interpolated_weighted.csv", "w") as f:
        for i in range(1, len(x)):
            f.write(f"{int(new_reference[0][i]*subdivisions)/subdivisions},{weighted(recorded_reference[1][i], new_reference[1][i])},{weighted(recorded_reference[2][i], new_reference[2][i])}\n")
    

def weighted(new,reference):
    w_new = 0.2
    w_ref = 0.8
    avg = (w_new*new+w_ref*reference)
    return int(avg)

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