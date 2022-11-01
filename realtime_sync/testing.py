import numpy as np
import time

t = np.array([[2, 3, 4, 5],
            [9, 2, 5, 5],
            [1, 2, 3, 5]])
data = t.transpose()
print(data)
print(data[:,0:1])
x = data[:,0:1].transpose()
x = np.append(x, data[:,1:2].transpose(), axis = 0)
print(x)
print(x[:, 0:1])
def interp_nd(x, rx, ry):

    """ Interpolate new subdivision points given as (x) on a given spread of time (rx) and points (ry) """

    #breakpoint()
    data = np.zeros((len(ry), len(x)))
    for i in range(len(ry)):
        data[i] = np.interp(x, rx, ry[i])
    return data
