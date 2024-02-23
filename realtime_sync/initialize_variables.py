from tempfile import TemporaryFile
import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit
import os
import sys
import getopt

sys.path.append("..")
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


ROBOT_MAIN = "192.168.65.244"  # actual robot
ROBOT_SIDE = "192.168.65.245"
# ROBOT_HOST = '192.168.56.102'   # virtual robot
ROBOT_PORT = 30004
robot_speed = 100
updateFrequency = 500

# connect to the robot
global con

con = rtde.RTDE(ROBOT_MAIN, ROBOT_PORT)
con_side = rtde.RTDE(ROBOT_SIDE, ROBOT_PORT)
con.connect()
con_side.connect()

# test connection (get controller version)
global cv
cv = con.get_controller_version()
print(f"controller main version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")
cv_side = con_side.get_controller_version()
print(f"controller side version: {cv_side[0]}.{cv_side[1]}.{cv_side[2]}.{cv_side[3]}")


global config
config = rtde_config.ConfigFile("cfg.xml")


# subscribe to the desired data
global state_names, state_types
state_names, state_types = config.get_recipe("state")
con.send_output_setup(state_names, state_types, frequency=updateFrequency)

# subscribe to the desired data for side con
global state_side_names, state_side_types
state_side_names, state_side_types = config.get_recipe("state")
con_side.send_output_setup(
    state_side_names, state_side_types, frequency=updateFrequency
)
# input bit for acknowledging cycle start
global input_67_names, input_67_types
input_67_names, input_67_types = config.get_recipe("in67")
global input_67
input_67 = con.send_input_setup(input_67_names, input_67_types)

# input bit for halting the process
global input_65_names, input_65_types
input_65_names, input_65_types = config.get_recipe("in65")
global input_65
input_65 = con.send_input_setup(input_65_names, input_65_types)

# input bit for halting the process side
global input_65_side_names, input_65_side_types
input_65_side_names, input_65_side_types = config.get_recipe("in65")
global input_65_side
input_65_side = con_side.send_input_setup(input_65_side_names, input_65_side_types)

# input bit for setting the speed
global input_66_names, input_66_types
input_66_names, input_66_types = config.get_recipe("in66")
global input_66
input_66 = con.send_input_setup(input_66_names, input_66_types)

# input bit for setting the speed side
global input_66_side_names, input_66_side_types
input_66_side_names, input_66_side_types = config.get_recipe("in66")
global input_66_side
input_66_side = con_side.send_input_setup(input_66_side_names, input_66_side_types)


# input int for setting the speed
global speed_int_names, speed_int_types
speed_int_names, speed_int_types = config.get_recipe("speed_int")
global speed_int
speed_int = con.send_input_setup(speed_int_names, speed_int_types)

# input int for setting the speed side
global speed_int_side_names, speed_int_side_types
speed_int_side_names, speed_int_side_types = config.get_recipe("speed_int")
global speed_int_side
speed_int_side = con_side.send_input_setup(speed_int_side_names, speed_int_side_types)

# input bit 68 for side con to signal that it is ready
global input_68_side_names, input_68_side_types
input_68_side_names, input_68_side_types = config.get_recipe("in68")
global input_68_side
input_68_side = con_side.send_input_setup(input_68_side_names, input_68_side_types)


if not con_side.send_start():
    print("failed to start data transfer side")
    sys.exit()

if not con.send_start():
    print("failed to start data transfer main")
    sys.exit()


# setting default values so the program can run
input_65.input_bit_register_65 = int(False)
con.send(input_65)

input_66.input_bit_register_66 = int(False)
con.send(input_66)

speed_int.input_int_register_25 = robot_speed
con.send(speed_int)

input_68_side.input_bit_register_68 = int(False)
con_side.send(input_68_side)

input_67.input_bit_register_67 = int(False)
con.send(input_67)

time.sleep(0.5)
input_68_side.input_bit_register_68 = int(False)
con_side.send(input_68_side)


# ##TEST
# input_68_side.input_bit_register_68 = int(True)
# con_side.send(input_68_side)
split = ["speed", "50"]
speed_int.input_int_register_25 = int(split[1])
speed_int_side.input_int_register_25 = int(split[1])
con.send(speed_int)
con_side.send(speed_int_side)
input_66.input_bit_register_66 = int(True)
input_66_side.input_bit_register_66 = int(True)
con.send(input_66)
con_side.send(input_66_side)
# time.sleep(1)
# print(f"Speed set to {split[1]}%")
# print(split[1])
time.sleep(0.1)
input_66.input_bit_register_66 = int(False)
input_66_side.input_bit_register_66 = int(False)
con.send(input_66)
con_side.send(input_66_side)

# con_side.send(input_68_side)
split = ["speed", "50"]
speed_int.input_int_register_25 = int(split[1])
speed_int_side.input_int_register_25 = int(split[1])
con.send(speed_int)
con_side.send(speed_int_side)
input_66.input_bit_register_66 = int(True)
input_66_side.input_bit_register_66 = int(True)
con.send(input_66)
con_side.send(input_66_side)
# time.sleep(1)
# print(f"Speed set to {split[1]}%")
# print(split[1])
time.sleep(0.1)
input_66.input_bit_register_66 = int(False)
input_66_side.input_bit_register_66 = int(False)
con.send(input_66)
con_side.send(input_66_side)

# con_side.send(input_68_side)
split = ["speed", "50"]
speed_int.input_int_register_25 = int(split[1])
speed_int_side.input_int_register_25 = int(split[1])
con.send(speed_int)
con_side.send(speed_int_side)
input_66.input_bit_register_66 = int(True)
input_66_side.input_bit_register_66 = int(True)
con.send(input_66)
con_side.send(input_66_side)
# time.sleep(1)
# print(f"Speed set to {split[1]}%")
# print(split[1])
time.sleep(0.1)
input_66.input_bit_register_66 = int(False)
input_66_side.input_bit_register_66 = int(False)
con.send(input_66)
con_side.send(input_66_side)

# i = 0
# while True:
#     i += 1
#     print(i)
#     split = ["speed", "50"]
#     speed_int.input_int_register_25 = int(split[1])
#     speed_int_side.input_int_register_25 = int(split[1])
#     con.send(speed_int)
#     con_side.send(speed_int_side)
#     # time.sleep(0.1)
#     input_66.input_bit_register_66 = int(True)
#     input_66_side.input_bit_register_66 = int(True)
#     con.send(input_66)
#     con_side.send(input_66_side)
#     # time.sleep(1)
#     # print(f"Speed set to {split[1]}%")
#     # print(split[1])
#     time.sleep(0.1)
#     input_66.input_bit_register_66 = int(False)
#     input_66_side.input_bit_register_66 = int(False)
#     con.send(input_66)
#     con_side.send(input_66_side)
#     # time.sleep(0.1)
#     state = con.receive()
#     state_side = con_side.receive()


con.send_pause()
con.disconnect()

print("Variables initialized.")
