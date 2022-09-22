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
        idxs[i] = idxmin + get_normalized_idx(ref[:,idxmin:idxmax], pts[:,i:i+1], method)
        lastrefidx = int(max(lastrefidx, idxs[i]))
        
    return idxs

def interp_nd(x, rx, ry):
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data
        
def get_normalized_t(ref, tref, pt, method="euclidean", spread=2, subdivisions=30):
    if method=="euclidean":
        get_distance = get_euclidean
    elif method=="manhattan":
        get_distance = get_manhattan
    else:
        return -1
    _,p = get_distance(ref-pt)
    idxl = max(0, p[0][0]-spread)
    idxh = min(len(ref[0]), p[0][0]+spread+2)
    didx = idxh-idxl

    samplepts = np.linspace(tref[idxl],tref[idxh-1], didx*subdivisions)
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:,idxl:idxh])
    _,pp = get_distance(subdivided-pt)
    return samplepts[pp[0][0]], p[0][0]
    
def get_normalized_t_array(ref, tref, pts, method="euclidean", spread=2, subdivisions=100):
    # shrink the observed interval to speed up the process and eliminate overlapping
    past = 10       # how far into the past the algorithm looks
    future = 10    # how far into the future the algorithm looks
    if (method != "euclidean") and (method != "manhattan"):
        print("incorrect method; must be euclidean or manhattan")
        return 0
    
    t = np.zeros(np.shape(pts)[1])
    idxs = np.zeros(np.shape(pts)[1])
    datalen = np.shape(pts)[1]
    refdatalen = np.shape(ref)[1]
    lastrefidx = 0                  # stores the ixd of the last assigned reference point, monotonoulsy increasing
    for i in range(datalen):
        idxmin = max(0,lastrefidx-past)
        idxmax = min(refdatalen,lastrefidx+future)
        #idxs[i] = idxmin + get_normalized_idx(ref[:,idxmin:idxmax], pts[:,i:i+1], method)
        #time, idx = get_normalized_t(ref, tref, pts[:,i:i+1], method, spread, subdivisions)
        time, idx = get_normalized_t(ref[:,idxmin:idxmax], tref[idxmin:idxmax], pts[:,i:i+1], method, spread, subdivisions)
        t[i] = time
        idxs[i] = idxmin + idx
        lastrefidx = int(max(lastrefidx, idxs[i]))
    t = t*100/tref[-1]
    return t

def main():
    tagline = "test"

    # read data from files
    tS1, axS1, pctS1, doS1 = read_data("pctVar_1655379891.csv")
    t5,  ax5,  pct5,  do5  = read_data("pct5_1654688396.csv")
    t10, ax10, pct10, do10 = read_data("pct10_1654687925.csv")
    t25, ax25, pct25, do25 = read_data("pct25_1654688047.csv")
    t33, ax33, pct33, do33 = read_data("pct33_1654688078.csv")
    t50, ax50, pct50, do50 = read_data("pct50_1654688148.csv")
    #t50, ax50, pct50, do50 = read_data("pct50_1654771712_perfect.csv")
    t67, ax67, pct67, do67 = read_data("pct66_1654688180.csv")
    t75, ax75, pct75, do75 = read_data("pct75_1654688197.csv")
    t90, ax90, pct90, do90 = read_data("pct90_1654688218.csv")
    t100,ax100,pct100,do100= read_data("pct100_1654688233.csv")
    #t100,ax100,pct100,do100= read_data("pct100_1654773570_perfect.csv")
    #t100,ax100,pct100,do100= read_data("pct50_1654688148.csv")
    
    
    # prepare data
    randomAmount = 0.001


    # normalized progress with interpolation
    if 1:
        fig, ax = plt.subplots()
        #plt.plot(t25*100/t25[-1], t100[get_normalized_idx_array(ax100, ax25, method="euclidean").astype(int)]*100/t100[-1], label="25(100)e")
        #plt.plot(t50*100/t50[-1], t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)]*100/t100[-1], label="50(100)e")
        
        plt.plot(t5*100/t5[-1], get_normalized_t_array(ax100, t100, ax5, method="euclidean"), label="5(100)es")
        plt.plot(t10*100/t10[-1], get_normalized_t_array(ax100, t100, ax10, method="euclidean"), label="10(100)es")
        plt.plot(t25*100/t25[-1], get_normalized_t_array(ax100, t100, ax25, method="euclidean"), label="25(100)es")
        plt.plot(t33*100/t33[-1], get_normalized_t_array(ax100, t100, ax33, method="euclidean"), label="33(100)es")
        plt.plot(t50*100/t50[-1], get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)es")
        plt.plot(t67*100/t67[-1], get_normalized_t_array(ax100, t100, ax67, method="euclidean"), label="67(100)es")
        plt.plot(t75*100/t75[-1], get_normalized_t_array(ax100, t100, ax75, method="euclidean"), label="75(100)es")
        plt.plot(t90*100/t90[-1], get_normalized_t_array(ax100, t100, ax90, method="euclidean"), label="90(100)es")
        plt.plot(tS1*100/tS1[-1], get_normalized_t_array(ax100, t100, axS1, method="euclidean"), label="S1(100)es")
        
        
        ax.legend()

        fig.canvas.manager.set_window_title("mapping")
        ax.set_xlabel("actual progress [%]")
        ax.set_ylabel("reference progress [%]")
        ax.set_title("Progress")
        plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
        plt.show(block = False)

    # error of normalized progress with interpolation
    if 1:
        fig, ax = plt.subplots()
        #plt.plot(t25*100/t25[-1], t100[get_normalized_idx_array(ax100, ax25, method="euclidean").astype(int)]*100/t100[-1], label="25(100)e")
        #plt.plot(t50*100/t50[-1], t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)]*100/t100[-1], label="50(100)e")
        
        plt.plot(t5*100/t5[-1], get_normalized_t_array(ax100, t100, ax5, method="euclidean")-t5*100/t5[-1], label="5(100)es")
        plt.plot(t10*100/t10[-1], get_normalized_t_array(ax100, t100, ax10, method="euclidean")-t10*100/t10[-1], label="10(100)es")
        plt.plot(t25*100/t25[-1], get_normalized_t_array(ax100, t100, ax25, method="euclidean")-t25*100/t25[-1], label="25(100)es")
        plt.plot(t33*100/t33[-1], get_normalized_t_array(ax100, t100, ax33, method="euclidean")-t33*100/t33[-1], label="33(100)es")
        plt.plot(t50*100/t50[-1], get_normalized_t_array(ax100, t100, ax50, method="euclidean")-t50*100/t50[-1], label="50(100)es")
        plt.plot(t67*100/t67[-1], get_normalized_t_array(ax100, t100, ax67, method="euclidean")-t67*100/t67[-1], label="67(100)es")
        plt.plot(t75*100/t75[-1], get_normalized_t_array(ax100, t100, ax75, method="euclidean")-t75*100/t75[-1], label="75(100)es")
        plt.plot(t90*100/t90[-1], get_normalized_t_array(ax100, t100, ax90, method="euclidean")-t90*100/t90[-1], label="90(100)es")
        
        ax.legend()

        fig.canvas.manager.set_window_title("mapping")
        ax.set_xlabel("actual progress [%]")
        ax.set_ylabel("reference progress [%]")
        ax.set_title("Progress")
        plt.savefig(f"{tagline}_tmp.png", bbox_inches='tight', dpi=300)
        plt.show(block = False)
    # progress with interpolation
    if 1:
        fig, ax = plt.subplots()
        """
        plt.plot(t25, t100[get_normalized_idx_array(ax100, ax25, method="euclidean").astype(int)]*100/t100[-1], label="25(100)e")
        plt.plot(t50, t100[get_normalized_idx_array(ax100, ax50, method="euclidean").astype(int)]*100/t100[-1], label="50(100)e")
        
        plt.plot(t25, get_normalized_t_array(ax100, t100, ax25, method="euclidean"), label="25(100)es")
        plt.plot(t50, get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)es")
        """

        plt.plot(t5, get_normalized_t_array(ax100, t100, ax5, method="euclidean"), label="5(100)es")
        plt.plot(t10, get_normalized_t_array(ax100, t100, ax10, method="euclidean"), label="10(100)es")
        plt.plot(t25, get_normalized_t_array(ax100, t100, ax25, method="euclidean"), label="25(100)es")
        plt.plot(t33, get_normalized_t_array(ax100, t100, ax33, method="euclidean"), label="33(100)es")
        plt.plot(t50, get_normalized_t_array(ax100, t100, ax50, method="euclidean"), label="50(100)es")
        plt.plot(t67, get_normalized_t_array(ax100, t100, ax67, method="euclidean"), label="67(100)es")
        plt.plot(t75, get_normalized_t_array(ax100, t100, ax75, method="euclidean"), label="75(100)es")
        plt.plot(t90, get_normalized_t_array(ax100, t100, ax90, method="euclidean"), label="90(100)es")
        plt.plot(tS1, get_normalized_t_array(ax100, t100, axS1, method="euclidean"), label="S1(100)es")
        
        ax.legend()

        fig.canvas.manager.set_window_title("mapping")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("progress [%]")
        ax.set_title("Progress")
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



