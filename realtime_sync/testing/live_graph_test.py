import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import psutil
import collections
from threading import Thread
import time

# start collections with zeros
cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))
# define and adjust figure
#fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
fig, axs = plt.subplots(2,2)
ax = axs[0][0]
ax1 = axs[1][0]
ax.set_facecolor('#DEDEDE')
ax1.set_facecolor('#DEDEDE')
# function to update the data
def my_function(i):
    # get data
    cpu.popleft()
    cpu.append(psutil.cpu_percent())
    ram.popleft()
    ram.append(psutil.virtual_memory().percent)
    # clear axis
    ax.cla()
    ax1.cla()
    # plot cpu
    ax.plot(cpu)
    ax.scatter(len(cpu)-1, cpu[-1])
    ax.text(len(cpu)-1, cpu[-1]+2, "{}%".format(cpu[-1]))
    ax.set_ylim(0,100)
    # plot memory
    ax1.plot(ram)
    ax1.scatter(len(ram)-1, ram[-1])
    ax1.text(len(ram)-1, ram[-1]+2, "{}%".format(ram[-1]))
    ax1.set_ylim(0,100)
    text_str1 = "sample_text"
    text_str = "other sample_text"
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax1.text(0.05, 0.95, text_str1, transform=ax1.transAxes, fontsize=14,
        verticalalignment='top', bbox=props)
    ax.text(0.05, 0.95, text_str, transform=ax.transAxes, fontsize=14,
        verticalalignment='top', bbox=props)
# animate
def output():
    while True:
        print("Printing text")
        time.sleep(1)

def main():
    t_term = Thread(target = output)
    t_term.start()
    ani = FuncAnimation(fig, my_function, interval=1000)
    plt.show()

if __name__ == "__main__":
    main()