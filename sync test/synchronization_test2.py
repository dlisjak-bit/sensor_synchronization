import matplotlib.pyplot as plt
import numpy as np
from perlin_noise import PerlinNoise
import timeit

def ax0_fcn(x):
    noise = PerlinNoise(octaves=.003*np.pi, seed=1)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def ax1_fcn(x):
    noise = PerlinNoise(octaves=.006*np.e, seed=2)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def ax2_fcn(x):
    noise = PerlinNoise(octaves=.005*np.sqrt(2), seed=3)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def ax3_fcn(x):
    noise = PerlinNoise(octaves=.01*np.sqrt(3), seed=4)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def ax4_fcn(x):
    noise = PerlinNoise(octaves=.1*np.sqrt(7), seed=5)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def ax5_fcn(x):
    noise = PerlinNoise(octaves=.1*np.sqrt(11), seed=6)
    if(isinstance(x,int) or isinstance(x,float)): return noise(x)
    fcn = [noise(i) for i in x]
    return np.array(fcn)

def axs_fcns(x):
    #if(isinstance(x,int) or isinstance(x,float)):
    #data = [ax0_fcn(x),ax1_fcn(x),ax2_fcn(x)]#,ax3_fcn(x),ax4_fcn(x),ax5_fcn(x)]
    data = [ax0_fcn(x),ax1_fcn(x),ax2_fcn(x),ax3_fcn(x)]#,ax4_fcn(x),ax5_fcn(x)]
    return np.array(data)

def get_manhattan(err):
    d_manhattan = np.sum(np.abs(err), axis=0)
    p_min_manhattan = np.where(d_manhattan == np.amin(d_manhattan))
    return d_manhattan, p_min_manhattan

def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean





def main():
    tagline = "test"
    
    # prepare data
    randomAmount = 0.001
    tmax = 100
    sampleRate = 125
    t = np.linspace(0,tmax,tmax*sampleRate)
    data = axs_fcns(t)

    # prepare (noisy) sampling point
    pt_t = 90
    pt_t = np.random.random_sample(1)[0]*tmax
    print(f"time: {pt_t*1}, closest sample: {int(np.round(pt_t*sampleRate))}")
    pt = np.array([axs_fcns(pt_t)])
    #np.random.seed(1)
    pt[0,:] = pt[0,:] + (np.random.random_sample(len(pt[0,:]))*2-1)*randomAmount

    # calculate best fit for pt in data
    err = data-pt.T # x.T is x.transpose()
    d_manhattan, p_min_manhattan = get_manhattan(err)
    print(f'manhattan min idx: {p_min_manhattan[0][0]}, target: {int(np.round(pt_t*sampleRate))}, dt: {(p_min_manhattan[0][0]-pt_t*sampleRate)/sampleRate} s')
    d_euclidean, p_min_euclidean = get_euclidean(err)
    print(f'euclidean min idx: {p_min_euclidean[0][0]}, target: {int(np.round(pt_t*sampleRate))}, dt: {(p_min_manhattan[0][0]-pt_t*sampleRate)/sampleRate} s')

    # draw functions
    fig, ax = plt.subplots()
    for i in range(len(data)):
        plt.plot(t, data[i], label=str(i))
        plt.plot(pt_t, pt[0,i], label=f"{i}.", linestyle='None', marker="x", markersize=5)
    
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
    plt.plot(t, d_manhattan, label="manhattan")
    plt.plot(t, d_euclidean, label="euclidean")
    ax.legend()

    fig.canvas.manager.set_window_title("distance")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance")
    ax.set_title("Euclidean and Manhattan Distance")
    plt.savefig(f"{tagline}_error.png", bbox_inches='tight')
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
    t_manhattan = timeit.Timer("get_manhattan(err)", setup=SETUP).repeat(3,1000)
    print(f"timeit(manhattan):\t{t_manhattan} ms")

    t_euclidean = timeit.Timer("get_euclidean(err)", setup=SETUP).repeat(3,1000)
    print(f"timeit(euclidean):\t{t_euclidean} ms")


    
if __name__ == "__main__":
    main()



