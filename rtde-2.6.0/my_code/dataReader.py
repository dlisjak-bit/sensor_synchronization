import sys
sys.path.append('..')
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import numpy as np
import matplotlib.pyplot as plt

# system variables
updateFrequency = 500
samplingTime = 60   #sampling time in seconds

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.65.178'   # virtual robot
ROBOT_PORT = 30004

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


# start the data synchronization
if not con.send_start():
    print("failed to start data transfer")
    sys.exit()
    

# the meat and potatoes of this script
ta = []
qa = []
doa = []

timestamp = str(int(time.time()))
with open(f"recordings/{timestamp}.csv", "w") as f:
    f.write("t,q0,q1,q2,q3,q4,q5,do\n") # write header row
    #while keep_running:
    print("Sampling started")
    for i in range(samplingTime):
        for j in range(updateFrequency):
            # receive the current state
            state = con.receive()
            
            if state is None:
                break;

            t = state.timestamp
            q = state.actual_q
            do = state.actual_digital_output_bits
            data = f"{t},{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]},{do}"

            ta.append(t)
            qa.append(q)
            doa.append(do)

            f.write(data+"\n")
        print(f"Recorded {i+1} of {samplingTime} s")
    #print(data)

con.send_pause()
con.disconnect()


# prepare and crop data
ta = np.array(ta)-ta[0]
qa = np.array(qa).transpose()
doa = np.array(doa)

z = np.where(doa == 0)
if len(z[0]):
    cycleStart_i = z[0][0]
    cycleStop_i = z[0][-1]
    while (doa[cycleStart_i] == 0): cycleStart_i += 1
    while (doa[cycleStop_i] == 0): cycleStop_i -= 1
    cycleStop_i += 1
else:
    cycleStart_i = 0
    cycleStop_i = len(ta)-1

# save cropped data
with open(f"recordings/{timestamp}_cropped.csv", "w") as f:
    f.write("t,q0,q1,q2,q3,q4,q5,do\n") # write header row
    for i in range(cycleStart_i,cycleStop_i):
        f.write(f"{ta[i]},{qa[0][i]},{qa[1][i]},{qa[2][i]},{qa[3][i]},{qa[4][i]},{qa[5][i]},{doa[i]}\n")

# data visualization
fig, ax = plt.subplots(figsize=(10,4))

# draw axis curves
for i, q in enumerate(qa):
    ax.plot(ta, q, label=f'ax{i}')
#ax.plot(ta, doa, label='do')
ax.plot(ta[1:], np.diff(ta)*100, label=f'dt')

# grey out data before and central repetition

plt.axvspan(ta[0], ta[cycleStart_i], facecolor='0.2', alpha=0.3)
plt.axvspan(ta[cycleStop_i], ta[-1], facecolor='0.2', alpha=0.3)

ax.legend()
fig.canvas.manager.set_window_title("reference capture")
ax.set_xlabel("time [s]")
ax.set_ylabel("axis rotation [rad]")
#ax.set_title("Standard Deviation")
#plt.savefig(f"{timestamp}_x.png", bbox_inches='tight')
plt.show(block = False)
