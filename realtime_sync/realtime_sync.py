from tempfile import TemporaryFile
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


# --------------GUIDE----------------------------------------
# run the script as python3 realtime_sync.py <-t> <-r>
# optional flags: 
# -t: show calculated reference time in commandline
# -r: record a reference loop before starting data collection



# system variables
updateFrequency = 125
samplingTime = 120   #sampling time in seconds
method = "euclidean"
reference_file = "reference_1664106886.csv"

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

argumentList = sys.argv[1:]

def new_reference():

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

def read_reference(file):

    """ Read reference file. """

    data = []
    with open(file, 'r') as f:
        f.readline()    # throw away the header line
        for l in f:
            #print(l.strip())
            data.append([float(x) for x in l.strip().split(',')])   # read csv
    data = np.array(data).transpose()                               # transpose matrix
                                                                    # separate data:
    return data[0]-data[0][0], data[1:13], data[-2], data[-1]       # time points array ([time]-[start time]), 12 arrays (q0 do qd5) , array tsf, array do

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

    if not con.send_start():
        print("failed to start data transfer")
        sys.exit()

def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan

def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean

def normalize(v):
    v=np.array(v)
    norm=np.linalg.norm(v)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

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
    #breakpoint()
    #print(p, idxl, idxh)

    # Allocate array to divide time into more sample points
    samplepts = np.linspace(tref[idxl],tref[idxh-1], didx*subdivisions)
    
    #subdivided = np.interp(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])

    # Interpolate the data between reference time points and fill samplepts array
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])

    # Find a more precise reference time estimate
    d,p = get_distance(subdivided-pt)
    return samplepts[p[0][0]]

def data_reader(tref, ref):

    """ Continuously read data from robot and find reference time for each point """
    #samplingState = "waiting for sync low"
    keep_running = True
    print("Sampling started")
    while keep_running:
        #for i in range(samplingTime):
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

            #if samplingState == "waiting for sync low":
                #if (do%2)==0:
                    #samplingState = "waiting for sync high"
            #elif samplingState == "waiting for sync high":
                #if (do%2)==1:
                    #print("cycle start detected")
                    #samplingState = "collecting data"
            #if samplingState == "collecting data":
                #if (do%2)==0:
                    #samplingState = "finished"
                    #break
            #datapt.append(t)
            datapt = np.append(q,qd)
            #datapt.extend(q)
            #datapt.extend(qd)
            #datapt.append(ss)
            #datapt.append(tsf)
            #datapt.append(do)

            datapt = np.array(datapt).transpose()
            datapt = datapt.reshape(12,1)
            normalized_t = get_normalized_t(ref, tref, datapt, method)
            if show_time:
                print(normalized_t, end="\r")
                 
            #print(f"Recorded {i+1} of {samplingTime} s")
            #if samplingState == "finished":
                #print("cycle complete")
                #break
    con.send_pause()
    con.disconnect()

def main(argumentList, reference_file):
    


    record_reference = False
    global show_time
    show_time = False

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

    connect_robot()
    if record_reference:
        reference_file = new_reference()
        print("Using new reference motion ...")
    tref,ref,pct100,do100 = read_reference(reference_file)
    data_reader(tref, ref)

main(argumentList, reference_file)