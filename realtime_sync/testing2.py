import numpy as np

array = []
for i in range(3):
    array.append([])
print(array)
returned = [2,3]
array[0] = returned
print(array)