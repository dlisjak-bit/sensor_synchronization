import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit

def read_data(file):
    data = []
    with open(file, 'r') as f:
        f.readline()    # throw away the header line
        for l in f:
            #print(l.strip())
            data.append([float(x) for x in l.strip().split(',')])
    data = np.array(data).transpose()
    return data[0]-data[0][0], data[1:7], data[7]

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
    if (method != "euclidean") and (method != "manhattan"):
        print("incorrect method; must be euclidean or manhattan")
        return 0
    
    idxs = np.zeros(np.shape(pts)[1])
    for i in range(np.shape(pts)[1]):
        idxs[i] = get_normalized_idx(ref, pts[:,i:i+1], method)
        #print(get_normalized_idx(ref, pts[:,i:i+1], method))
        #foo = get_normalized_idx(ref, pts[:,i:i+1], method)
        #float("sdg")
    return idxs

def main():
    tagline = "test"

    # read data from files
    t25, ax25, do25 = read_data("pct25_1654070580.csv")
    t33, ax33, do33 = read_data("pct33_1654070507.csv")
    t50, ax50, do50 = read_data("pct50_1654068682.csv")
    t66, ax66, do66 = read_data("pct66_1654068584.csv")
    t75, ax75, do75 = read_data("pct75_1654068509.csv")
    t100,ax100,do100 =read_data("pct100_1654065927.csv")
    
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
        plt.plot(t25, ax25[i], label=str(i))
        #plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    
    #plt.plot(5, ax5_fcn(5), label="5.", linestyle='None', marker=".", markersize=5)
    ax.legend()
    fig.canvas.manager.set_window_title("error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.set_title("Axis Functions")
    plt.savefig(f"{tagline}fcns.png", bbox_inches='tight')
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


    # bla
    fig, ax = plt.subplots()

    plt.plot(get_normalized_idx_array(ax100, ax25, method="euclidean"), label="25(100)e")
    plt.plot(get_normalized_idx_array(ax100, ax50, method="euclidean"), label="50(100)e")

    plt.plot(get_normalized_idx_array(ax75, ax25, method="euclidean"), label="25(75)e")
    plt.plot(get_normalized_idx_array(ax75, ax50, method="euclidean"), label="50(75)e")
    
    plt.plot(get_normalized_idx_array(ax66, ax25, method="euclidean"), label="25(66)e")
    plt.plot(get_normalized_idx_array(ax66, ax50, method="euclidean"), label="50(66)e")
    
    ax.legend()

    fig.canvas.manager.set_window_title("mapping")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("progress [%]")
    ax.set_title("Progress")
    #plt.savefig(f"{tagline}_progress.png", bbox_inches='tight')
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

    
if __name__ == "__main__":
    main()



