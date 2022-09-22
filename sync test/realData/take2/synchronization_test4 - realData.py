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
    #return data[0]-data[0][0], data[1:7], data[-2], data[-1]

def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan

def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean

def get_normalized_idx(ref, pt, method="euclidean"):
    if method=="euclidean":
        d,p = get_euclidean(ref-pt)
        return p[0]
    elif method=="manhattan":
        d,p = get_manhattan(ref-pt)
        return p[0]
    else:
        return -1

def get_normalized_idx_array(ref, pts, method="euclidean"):
    # shrink the observed interval to speed up the process and eliminate overlapping
    past = 100       # how far into the past the algorithm looks
    future = 100    # how far into the future the algorithm looks
    if (method != "euclidean") and (method != "manhattan"):
        print("incorrect method; must be euclidean or manhattan")
        return 0
    
    idxs = np.zeros(np.shape(pts)[1])
    datalen = np.shape(pts)[1]
    refdatalen = np.shape(ref)[1]
    lastrefidx = 0                  # stores the ixd of the last assigned reference point, monotonoulsy increasing
    for i in range(datalen):
        idxmin = max(0,lastrefidx-past)
        idxmax = min(refdatalen,lastrefidx+future)
        #print(idxmin, idxmax)
        idxs[i] = idxmin + get_normalized_idx(ref[:,idxmin:idxmax], pts[:,i:i+1], method)
        lastrefidx = int(max(lastrefidx, idxs[i]))
        
    return idxs

def main():
    tagline = "test"

    # read data from files
    t5,  ax5,  pct5,  do5  = read_data("pct5_1654688396.csv")
    t10, ax10, pct10, do10 = read_data("pct10_1654687925.csv")
    t25, ax25, pct25, do25 = read_data("pct25_1654688047.csv")
    t33, ax33, pct33, do33 = read_data("pct33_1654688078.csv")
    t50, ax50, pct50, do50 = read_data("pct50_1654688148.csv")
    t67, ax67, pct67, do67 = read_data("pct66_1654688180.csv")
    t75, ax75, pct75, do75 = read_data("pct75_1654688197.csv")
    t90, ax90, pct90, do90 = read_data("pct90_1654688218.csv")
    t100,ax100,pct100,do100 =read_data("pct100_1654688233.csv")
    
    # prepare data
    randomAmount = 0.001


    """
    # calculate best fit for pt in data
    err = data-pt.T # x.T is x.transpose()
    d_manhattan, p_min_manhattan = get_manhattan(err)
    print(f'manhattan min idx: {p_min_manhattan[0][0]}, target: {int(np.round(pt_t*sampleRate))}, dt: {(p_min_manhattan[0][0]-pt_t*sampleRate)/sampleRate} s')
    d_euclidean, p_min_euclidean = get_euclidean(err)
    print(f'euclidean min idx: {p_min_euclidean[0][0]}, target: {int(np.round(pt_t*sampleRate))}, dt: {(p_min_manhattan[0][0]-pt_t*sampleRate)/sampleRate} s')
    """
    # draw functions
    fig, ax = plt.subplots()
    for i in range(len(ax25)):
        plt.plot(t25, ax25[i], linestyle='none', marker='*', label=f"25%-{i}")
        #plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    for i in range(len(ax100)):
        plt.plot(t100, ax100[i], linestyle='none', marker='x', label=f"100%-{i}")
    
    #plt.plot(5, ax5_fcn(5), label="5.", linestyle='None', marker=".", markersize=5)
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.set_title("Axis Functions")
    #plt.savefig(f"{tagline}fcns.png", bbox_inches='tight')
    plt.show(block = False)
    
    # draw functions
    fig, ax = plt.subplots()
    for i in range(len(ax25)):
        plt.plot(t25/t25[-1]*100, ax25[i], linestyle='none', marker='*', label=f"25%-{i}")
        #plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    for i in range(len(ax100)):
        plt.plot(t100/t100[-1]*100, ax100[i], linestyle='none', marker='x', label=f"100%-{i}")
    
    #plt.plot(5, ax5_fcn(5), label="5.", linestyle='None', marker=".", markersize=5)
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("progress [%]")
    ax.set_ylabel("value")
    ax.set_title("Normalized Axis Functions")
    #plt.savefig(f"{tagline}fcns.png", bbox_inches='tight')
    plt.show(block = False)
    
    
    """
    # draw errors
    fig, ax = plt.subplots()
    for i in range(len(data)):
        plt.plot(t, err[i], label=str(i))
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error")
    ax.set_title("Error")
    plt.savefig(f"{tagline}_distance.png", bbox_inches='tight')
    plt.show(block = False)
    """
    """
    # draw manhattan distance
    fig, ax = plt.subplots()
    pt = np.array([[0,0,0,0,0,0]]).T
    pt = np.array([[-1.59,-1.72,-2.2,-0.84,1.60,-.02]]).T
    plt.plot(t25, get_manhattan(ax25-pt)[0], label="manhattan")
    plt.plot(t25, get_euclidean(ax25-pt)[0], label="euclidean")
    ax.legend()

    fig.canvas.manager.set_window_title("distance")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance")
    ax.set_title("Euclidean and Manhattan Distance")
    plt.savefig(f"{tagline}_error.png", bbox_inches='tight')
    plt.show(block = False)
    """
    """
    # draw progress
    fig, ax = plt.subplots()
    plt.plot(t25, t25/t25[-1]*100, label="25%")
    plt.plot(t33, t33/t33[-1]*100, label="33%")
    plt.plot(t50, t50/t50[-1]*100, label="50%")
    plt.plot(t66, t66/t66[-1]*100, label="66%")
    plt.plot(t75, t75/t75[-1]*100, label="75%")
    plt.plot(t100, t100/t100[-1]*100, label="100%")
    ax.legend()

    fig.canvas.manager.set_window_title("progress")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("progress [%]")
    ax.set_title("Progress")
    plt.savefig(f"{tagline}_progress.png", bbox_inches='tight')
    plt.show(block = False)
    """

    # bla
    fig, ax = plt.subplots()

    plt.plot(get_normalized_idx_array(ax100, ax25, method="euclidean"), label="25(100)e")
    plt.plot(get_normalized_idx_array(ax100, ax50, method="euclidean"), label="50(100)e")

    #plt.plot(get_normalized_idx_array(ax100, ax25, method="manhattan"), linestyle='dashed', label="25(100)e")
    #plt.plot(get_normalized_idx_array(ax100, ax50, method="manhattan"), linestyle='dashed', label="50(100)e")

    #plt.plot(get_normalized_idx_array(ax75, ax25, method="euclidean"), label="25(75)e")
    #plt.plot(get_normalized_idx_array(ax75, ax50, method="euclidean"), label="50(75)e")
    
    #plt.plot(get_normalized_idx_array(ax67, ax25, method="euclidean"), label="25(67)e")
    #plt.plot(get_normalized_idx_array(ax67, ax50, method="euclidean"), label="50(67)e")

    plt.plot(get_normalized_idx_array(ax25, ax5, method="euclidean"), label="5(25)e")
    plt.plot(get_normalized_idx_array(ax25, ax10, method="euclidean"), label="10(25)e")
    plt.plot(get_normalized_idx_array(ax25, ax25, method="euclidean"), label="25(25)e")
    plt.plot(get_normalized_idx_array(ax25, ax33, method="euclidean"), label="33(25)e")
    plt.plot(get_normalized_idx_array(ax10, ax33, method="euclidean"), label="33(10)e")

    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)

    
    """
    # draw first three axis in 3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(data[0],data[1],data[2], label = "reference")
    ax.plot(err[0],err[1],err[2], label = "error")
    ax.legend()
    plt.show(block = False)
    """
    
    # timeit stuff
    SETUP = """
import numpy as np
from __main__ import axs_fcns, get_manhattan, get_euclidean
        
t = np.linspace(0,100,1000)
data = axs_fcns(t)

pt_t = 17
pt = np.array([axs_fcns(pt_t)])
np.random.seed(1)
pt[0,:] = pt[0,:] + np.random.random(len(pt))*0.1
err = data-pt.T # x.T is x.transpose()
"""
    """
    t_manhattan = timeit.Timer("get_manhattan(err)", setup=SETUP).repeat(3,1000)
    print(f"timeit(manhattan):\t{t_manhattan} ms")

    t_euclidean = timeit.Timer("get_euclidean(err)", setup=SETUP).repeat(3,1000)
    print(f"timeit(euclidean):\t{t_euclidean} ms")
    """
    # bla
    fig, ax = plt.subplots()
    plt.plot(t25/t25[-1], t100[get_normalized_idx_array(ax100, ax25, method="euclidean").astype(int)], label="25(100)e")
    plt.plot(t50/t50[-1], t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)], label="50(100)e")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
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



