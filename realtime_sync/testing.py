import numpy as np
import time

t = [2, 3, 5, 6, 7, 20]
t_sample_start = 4.5

t_dif = [abs(t_sample_start-point) for point in t]

print(t_dif.index(min(t_dif)))