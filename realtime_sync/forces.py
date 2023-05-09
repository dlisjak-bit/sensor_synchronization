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


ROBOT_HOST = "192.168.65.244"  # actual robot
# ROBOT_HOST = "192.168.56.102"  # virtual robot
ROBOT_PORT = 30004
robot_speed = 100
updateFrequency = 125

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
con.send_output_setup(state_names, state_types, frequency=updateFrequency)

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

np.set_printoptions(
    edgeitems=5, linewidth=1000, formatter=dict(float=lambda x: "%.12f" % x)
)

if not con.send_start():
    print("failed to start data transfer")
    sys.exit()
force = con.receive()
avg = np.array(force.target_current).astype(float)
memory = [[], []]
actual_forces = [[], []]
start_time = time.time()
force = con.receive()
actual_forces[0].append(np.array(force.actual_current))
actual_forces[1].append(time.time() - start_time)
for i in range(20):
    for j in range(updateFrequency):
        # receive the current state
        state = con.receive()
        target = np.array(state.target_current).astype(float)
        actual = np.array(state.actual_current).astype(float)
        # print(np.shape(target), np.shape(actual))
        dif = (actual - target) / target
        # dif = abs(target - actual)
        print(dif, end="\r")
        # avg = (avg + np.array(force.ft_raw_wrench).astype(float)) / 2
        memory[0].append(dif)
        memory[1].append(time.time() - start_time)
        actual_forces[0].append(np.array(actual))
        actual_forces[1].append(time.time() - start_time)
with open("forces_test.csv", "w") as f:
    for i in range(len(memory[0])):
        f.write(f"{memory[0][i]},{memory[1][i]}\n")

fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(memory[1], memory[0])
ax2.plot(actual_forces[1], actual_forces[0])
ax1.set_xlabel("time")
ax1.set_ylabel("relative force deviation from prev measurement")
ax1.legend(range(6))
ax2.set_xlabel("time")
ax2.set_ylabel("absolute force measurement")
plt.show()
