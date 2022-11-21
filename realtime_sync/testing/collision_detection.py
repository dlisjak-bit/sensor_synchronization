import numpy as np
import time
times = np.array([0.5, 1.0, 1.5, 1.49, 0.2, 0.1])
print(max(times))
print(np.where(times == max(times))[0][0])
times_old = times[0:np.where(times == max(times))[0][0]+1]
print(times_old)