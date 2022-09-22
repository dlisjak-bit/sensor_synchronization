import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit
import os
#import pdb; pdb.set_trace()

def read_data(file):
    data = []
    with open(file, 'r') as f:
        f.readline()    # throw away the header line
        for l in f:
            #print(l.strip())
            data.append([float(x) for x in l.strip().split(',')])   # read csv
    data = np.array(data).transpose()                               # transpose matrix
                                                                    # separate data:
    return data[0]-data[0][0], data[1:13], data[-2], data[-1]       # time points array ([time]-[start time]), 12 arrays (q0 do qd5) , array tsf, array do
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

def interp_nd(x, rx, ry):

    """ Interpolate new subdivision points given as (x) on a given spread of time (rx) and points (ry) """

    #breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data
        
def get_normalized_t(ref, tref, pt, method="euclidean", spread=2, subdivisions=10):

    """ Use given method (euclidean/manhattan) to find nearest point in reference sample, interpolate nearby points to find precise reference time """

    if method=="euclidean":
        get_distance = get_euclidean
    elif method=="manhattan":
        get_distance = get_manhattan
    else:
        return -1
    d,p = get_distance(ref-pt)
    idxl = max(0, p[0][0]-spread)
    idxh = min(len(ref[0]), p[0][0]+spread+2)
    didx = idxh-idxl
    #breakpoint()
    #print(p, idxl, idxh)

    # Allocate array to divide time into more sample points
    samplepts = np.linspace(tref[idxl],tref[idxh-1], didx*subdivisions)
    
    #subdivided = np.interp(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])

    # Interpolate the data between reference time points and fill samplepts array
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])

    # Find a more precise reference time estimate
    d,p = get_distance(subdivided-pt)
    return samplepts[p[0][0]]
    
def get_normalized_t_array(ref, tref, pts, method="euclidean", spread=2, subdivisions=10):

    """  Assign every point in time to its appropriate reference point using get_normalized_t() """

    # shrink the observed interval to speed up the process and eliminate overlapping
    past = 100       # how far into the past the algorithm looks
    future = 100     # how far into the future the algorithm looks
    if (method != "euclidean") and (method != "manhattan"):
        print("incorrect method; must be euclidean or manhattan")
        return 0
    
    t = np.zeros(np.shape(pts)[1])
    datalen = np.shape(pts)[1]
    refdatalen = np.shape(ref)[1]
    #lastrefidx = 0                  # stores the ixd of the last assigned reference point, monotonoulsy increasing
    for i in range(datalen):
        #idxmin = max(0,lastrefidx-past)
        #idxmax = min(refdatalen,lastrefidx+future)
        #print(idxmin, idxmax)
        #breakpoint()
        #if i == 44: breakpoint()
        foo = get_normalized_t(ref, tref, pts[:,i:i+1], method)
        #print(i, foo)
        #if i == 52: breakpoint()
        t[i] = foo
        #lastrefidx = int(max(lastrefidx, idxs[i]))
        
    return t

def main():
    tagline = "test"

    # read data from files
    t25a, ax25a, pct25a, do25a = read_data("pct25_1654772804.csv")
    t25b, ax25b, pct25b, do25b = read_data("pct25_1654773379.csv")
    t50, ax50, pct50, do50 = read_data("pct50_1654771712.csv")
    t100,ax100,pct100,do100 =read_data("pct100_1654773570.csv")
    
    # prepare data
    randomAmount = 0.001


    # bla
    fig, ax = plt.subplots()

    plt.plot(get_normalized_idx_array(ax100, ax25a, method="euclidean"), label="25a(100)e")
    plt.plot(get_normalized_idx_array(ax100, ax25b, method="euclidean"), linestyle='dashed', label="25b(100)e")
    plt.plot(get_normalized_idx_array(ax100, ax50, method="euclidean"), label="50(100)e")

    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)

    # bla
    fig, ax = plt.subplots()
    plt.plot(t25a/t25a[-1], t100[get_normalized_idx_array(ax100, ax25a, method="euclidean").astype(int)], label="25a(100)e")
    plt.plot(t25b/t25b[-1], t100[get_normalized_idx_array(ax100, ax25b, method="euclidean").astype(int)], linestyle='dashed', label="25b(100)e")
    plt.plot(t50/t50[-1], t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)], label="50(100)e")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)




    # bla
    fig, ax = plt.subplots()
    plt.plot(t25a/t25a[-1], get_normalized_t_array(ax100, t100, ax25a, method="euclidean"), label="25a(100)e")
    plt.plot(t25b/t25b[-1], get_normalized_t_array(ax100, t100, ax25b, method="euclidean"), linestyle='dashed', label="25b(100)e")
    plt.plot(t50/t50[-1], get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)e")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)

    # normalized progress with interpolation
    fig, ax = plt.subplots()
    plt.plot(t25a/t25a[-1], t100[get_normalized_idx_array(ax100, ax25a, method="euclidean").astype(int)], label="25a(100)e")
    plt.plot(t25b/t25b[-1], t100[get_normalized_idx_array(ax100, ax25b, method="euclidean").astype(int)], linestyle='dashed', label="25b(100)e")
    plt.plot(t50/t50[-1], t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)], label="50(100)e")
    
    plt.plot(t25a/t25a[-1], get_normalized_t_array(ax100, t100, ax25a, method="euclidean"), label="25a(100)es")
    plt.plot(t25b/t25b[-1], get_normalized_t_array(ax100, t100, ax25b, method="euclidean"), linestyle='dashed', label="25b(100)es")
    plt.plot(t50/t50[-1], get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)es")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    #ax.set_xlabel("time [s]")
    #ax.set_ylabel("progress [%]")
    #ax.set_title("Progress")
    #plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)

    # progress with interpolation
    fig, ax = plt.subplots()
    plt.plot(t25a, t100[get_normalized_idx_array(ax100, ax25a, method="euclidean").astype(int)], label="25a(100)e")
    plt.plot(t25b, t100[get_normalized_idx_array(ax100, ax25b, method="euclidean").astype(int)], linestyle='dashed', label="25b(100)e")
    plt.plot(t50, t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)], label="50(100)e")
    
    plt.plot(t25a, get_normalized_t_array(ax100, t100, ax25a, method="euclidean"), label="25a(100)es")
    plt.plot(t25b, get_normalized_t_array(ax100, t100, ax25b, method="euclidean"), linestyle='dashed', label="25b(100)es")
    plt.plot(t50, get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)es")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("progress [%]")
    ax.set_title("Progress")
    #plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
    plt.show(block = False)
    



    # draw functions
    fig, ax = plt.subplots()
    for i in range(len(ax25a)):
        plt.plot(t25a, ax25a[i], label=str(i))
        #plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    
    #plt.plot(5, ax5_fcn(5), label="5.", linestyle='None', marker=".", markersize=5)
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.set_title("Axis Functions")
    plt.savefig(f"{tagline}fcns.png", bbox_inches='tight')
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



