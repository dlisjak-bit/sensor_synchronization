import sys
sys.path.append('..')
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import numpy as np
import matplotlib.pyplot as plt
import os

# system variables
updateFrequency = 125
samplingTime = 120   #sampling time in seconds

ROBOT_HOST = '192.168.65.244'   # actual robot
ROBOT_HOST = '192.168.56.101'   # virtual robot
ROBOT_PORT = 30004

# connect to the robot
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# test connection (get controller version)
cv = con.get_controller_version()
print(f"controller version: {cv[0]}.{cv[1]}.{cv[2]}.{cv[3]}")

# subscribe to the desired data
config = rtde_config.ConfigFile("cfg2.xml")
state_names, state_types = config.get_recipe("state")
con.send_output_setup(state_names, state_types, frequency = updateFrequency)


# start the data synchronization
if not con.send_start():
    print("failed to start data transfer")
    sys.exit()
    
def normalize(v):
    v=np.array(v)
    norm=np.linalg.norm(v)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

# the meat and potatoes of this script
ta = []
qa = []
qda = []
ssa = []
tsfa = []
doa = []

timestamp = str(int(time.time()))
ogfilename = f"recordings2/{timestamp}.csv"
with open(ogfilename, "w") as f:
    f.write("t,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,ss,tsf,do\n") # write header row

    samplingState = "waiting for sync low"
    #while keep_running:
    print("Sampling started")
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
                ta.append(t)
                qa.append(q)
                qda.append(qd)
                ssa.append(ss)
                tsfa.append(tsf)
                doa.append(do)
                data = f"{t},{str(q)[1:-1]},{str(qd)[1:-1]},{ss},{tsf},{do}"
                f.write(data+"\n")
                print("data")
        print(samplingState)        
        print(f"Recorded {i+1} of {samplingTime} s")
        if samplingState == "finished":
            print("cycle complete")
            break
    #print(data)

con.send_pause()
con.disconnect()

if samplingState != "finished":
    print("WARNING: Program timed out before robot cycle was complete.")

# prepare data
print(qa)
ta = np.array(ta)-ta[0]
qa = np.array(qa).T
qda= np.array(qda).T
ssa= np.array(ssa)
tsfa=np.array(tsfa)
doa = np.array(doa)

# rename savefile based on execution speed
tsfavg = np.average(tsfa)
if (round(tsfavg,3) == tsfa[0]):
    os.rename(ogfilename, f"recordings2/pct{int(tsfa[0]*100)}_{timestamp}.csv")
else:
    os.rename(ogfilename, f"recordings2/pctVar_{timestamp}.csv")
    
# data visualization
fig, ax = plt.subplots(figsize=(10,4))

# draw axis curves
for i, q in enumerate(qa):
    ax.plot(ta, q, marker='*', label=f'q{i}')

# draw joint velocity curves
for i, q in enumerate(qda):
    ax.plot(ta, q, marker='*', label=f'qd{i}')

#ax.plot(ta, ssa, marker='*', label='ss')
ax.plot(ta, tsfa, marker='*', label='tsf')

ax.plot(ta[1:], np.diff(ta)*100, label='dt')

#ax.plot(ta, doa, label='do')


ax.legend()
fig.canvas.manager.set_window_title("reference capture")
ax.set_xlabel("time [s]")
ax.set_ylabel("various values")
#ax.set_title("Standard Deviation")
plt.savefig(f"{timestamp}_x.png", bbox_inches='tight')
plt.show(block = False)
