import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit
import os


def read_data(file):
    data = []
    with open(file, 'r') as f:
        f.readline()    # throw away the header line
        for l in f:
            #print(l.strip())
            data.append([float(x) for x in l.strip().split(',')])
    data = np.array(data).transpose()
    return data[0]-data[0][0], data[1:13], data[-2], data[-1]

def main():
    t25a, ax25a, pct25a, do25a = read_data("pct25_1654772804.csv")
    t25b, ax25b, pct25b, do25b = read_data("pct25_1654773379.csv")
    t50, ax50, pct50, do50 = read_data("pct50_1654771712.csv")
    t100,ax100,pct100,do100 =read_data("domentest2.csv")
    
    tagline = "domen"

      # draw functions
    fig, ax = plt.subplots()
    for i in range(len(ax25a)):
        plt.plot(t100, ax100[i], label=str(i))
        #plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    
    #plt.plot(5, ax5_fcn(5), label="5.", linestyle='None', marker=".", markersize=5)
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.set_title("Axis Functions")
    plt.savefig(f"{tagline}fcns_domen.png", bbox_inches='tight')
    plt.show(block = False)
    
if __name__ == "__main__":
    main()
    pass
    """
    files = list(os.walk("."))[-1][-1]
    for _,_,files in os.walk("."):
        for file in files:
            print(file)
    """