import numpy as np
import time
array1 = np.zeros((2,10))
print(array1)
print(np.delete(array1, 0, 1))
array2 = np.zeros((1,2))
array2 = np.transpose(array2)
print(np.hstack((array1,array2)))