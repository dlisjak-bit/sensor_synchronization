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


ROBOT_HOST = '192.168.65.244'   # actual robot
#ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004
robot_speed = 100
updateFrequency = 125

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

con.send_pause()
con.disconnect()

print("Variables initialized.")
