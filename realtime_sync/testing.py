import numpy as np
import time

msmnts = np.zeros((3,16),np.uintc)
print(msmnts)

timestamp = str(int(time.time()))
ogfilename = f"sensor_reference_test.csv"
with open(ogfilename, "w") as f:
    f.write("t,")
    for i in range(16):
        f.write(f"detection{i},")
    for i in range(16):
        f.write(f"status{i},")
    for i in range(15):
        f.write(f"distance{i},")
    f.write("distance15\n")