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
global force_name, force_type
force_name, force_type = config.get_recipe("forces")
con.send_output_setup(force_name, force_type, frequency=updateFrequency)
np.set_printoptions(
    edgeitems=5, linewidth=1000, formatter=dict(float=lambda x: "%.12f" % x)
)

if not con.send_start():
    print("failed to start data transfer")
    sys.exit()
force = con.receive()
avg = np.array(force.ft_raw_wrench).astype(float)
memory = [[], []]
actual_forces = [[], []]
start_time = time.time()
force = con.receive()
actual_forces[0].append(np.array(force.ft_raw_wrench))
actual_forces[1].append(time.time() - start_time)
for i in range(5):
    for j in range(updateFrequency):
        # receive the current state
        force = con.receive()
        dif = abs(
            np.array(force.ft_raw_wrench).astype(float) - actual_forces[0][-1]
        )
        print(dif, end="\r")
        avg = (avg + np.array(force.ft_raw_wrench).astype(float)) / 2
        memory[0].append(dif)
        memory[1].append(time.time() - start_time)
        actual_forces[0].append(np.array(force.ft_raw_wrench))
        actual_forces[1].append(time.time() - start_time)
with open("forces_test.csv", "w") as f:
    for i in range(len(memory[0])):
        f.write(f"{memory[0][i]},{memory[1][i]}\n")

fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(memory[1], memory[0])
ax2.plot(actual_forces[1], actual_forces[0])
ax1.set_xlabel("time")
ax1.set_ylabel("absolute force deviation from prev measurement")
ax1.legend(range(6))
ax2.set_xlabel("time")
ax2.set_ylabel("absolute force measurement")
plt.show()
