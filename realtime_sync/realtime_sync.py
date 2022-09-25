from tempfile import TemporaryFile
import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit
import os
import sys
sys.path.append('..')
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# system variables
updateFrequency = 125
samplingTime = 120   #sampling time in seconds
method = "euclidean"

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

def read_reference(file):
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

def data_reader():
    samplingState = "waiting for sync low"
    keep_running = True
    i = 0
    while keep_running:
        print("Sampling started")
        #for i in range(samplingTime):
        for j in range(updateFrequency):

            # receive the current state
            state = con.receive()
        
            if state is None:
                print("connection lost, breaking")
                break
        
            datapt = []
            t = state.timestamp
            q = state.target_q
            qd = list(normalize(state.target_qd))
            ss = state.speed_scaling
            tsf = state.target_speed_fraction
            do = state.actual_digital_output_bits

            # print(do)

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
                datapt += t + q + qd + ss + tsf + do
                datapt = np.array(datapt).transpose()
                print(datapt)
                #normalized_t = get_normalized_t(ref, tref, datapt, method)
                #print(normalized_t)
            
                
        print(samplingState)
        i += 1
        print(i)        
            #print(f"Recorded {i+1} of {samplingTime} s")
            #if samplingState == "finished":
                #print("cycle complete")
                #break



def main():
    #record_reference option?
    connect_robot()
    t100,ax100,pct100,do100 = read_reference("pct100_1654773570.csv")
    global ref, tref
    ref = ax100
    tref = t100
    data_reader()

main()