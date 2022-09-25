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

# system variables
updateFrequency = 125
samplingTime = 120   #sampling time in seconds
method = "euclidean"
reference_file = "reference_1664106886.csv"

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

global con
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# test connection (get controller version)
global cv
cv = con.get_controller_version()
print(f"controller version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")
config = rtde_config.ConfigFile("cfg.xml")
global state_names, state_types 
state_names, state_types = config.get_recipe("state")
con.send_output_setup(state_names, state_types, frequency = updateFrequency)

if not con.send_start():
    print("failed to start data transfer")
    sys.exit()

input_names, input_types = config.get_recipe("in")

input_data = con.send_input_setup(input_names, input_types)

finish_capture=True
camera_ready=False

input_data.input_bit_register_64 = int(finish_capture)
input_data.input_bit_register_65 = int(camera_ready)

con.send(input_data)