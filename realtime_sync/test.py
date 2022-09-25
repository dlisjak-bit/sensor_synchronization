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

# connect to the robot
global con
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# test connection (get controller version)
global cv
cv = con.get_controller_version()
print(f"controller version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")

# subscribe to the desired data
config = rtde_config.ConfigFile("cfg.xml")
state_names, state_types = config.get_recipe("state")


setspeed_names, setspeed_types = config.get_recipe("setspeed")
setspeed = con.send_input_setup(setspeed_names, setspeed_types)

speedcontrol_names, speedcontrol_types = config.get_recipe("speedcontrol")
speedcontrol = con.send_input_setup(speedcontrol_names, speedcontrol_types)

digout_names, digout_types = config.get_recipe("digout")
digout = con.send_input_setup(digout_names, digout_types)

con.send_output_setup(state_names, state_types, frequency = updateFrequency)



if not con.send_start():
    print("failed to start data transfer")
    sys.exit()

speedcontrol.speed_slider_mask = np.uint32(1)
con.send(speedcontrol)


setspeed.speed_slider_fraction = np.double(0.5)
con.send(setspeed)
state = con.receive()
tsf = state.target_speed_fraction
print(tsf)

print(digout)
digout.configurable_digital_output = 0x1101e1ca0
con.send(digout)
print(digout)

con.send(setspeed)
