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

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004




def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan

def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean

def connect_robot():
    # connect to the robot
    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
    con.connect()

    # test connection (get controller version)
    cv = con.get_controller_version()
    print(f"controller version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")

    # subscribe to the desired data
    config = rtde_config.ConfigFile("cfg.xml")
    state_names, state_types = config.get_recipe("state")
    con.send_output_setup(state_names, state_types, frequency = updateFrequency)



def main():
    connect_robot()