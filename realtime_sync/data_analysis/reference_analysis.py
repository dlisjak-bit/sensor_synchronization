import numpy as np
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from matplotlib.colors import LinearSegmentedColormap

with open("alldata/box_experiment/adapted_reference3.csv", "r") as file:
    lines = file.readlines()


# Initialize lists to store data
cycle_number = []
time_in_cycle = []
reference0 = []
reference1 = []
prev_time_in_cycle = 0
current_cycle_number = 0
for line in lines:
    values = line.strip().split(",")

    if float(values[0]) < prev_time_in_cycle:
        current_cycle_number += 1
    reference0.append(float(values[1]))
    reference1.append(float(values[2]))
    time_in_cycle.append(float(values[0]))
    cycle_number.append(current_cycle_number)
    prev_time_in_cycle = float(values[0])

# cycle_number = np.array(cycle_number)
# time_in_cycle = np.array(time_in_cycle)
# reference0 = np.array(reference0)
# reference1 = np.array(reference1)
curves_array = []
current_array = []
current_cycle_number = 0
for i in range(len(reference0)):
    if cycle_number[i] != current_cycle_number:
        curves_array.append(current_array)
        current_array = []
        current_cycle_number += 1
    current_array.append(
        [time_in_cycle[i], reference0[i], reference1[i], current_cycle_number]
    )
# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")


# # Define the colormap and normalize it based on the range of data
# cmap = plt.get_cmap("viridis")  # You can choose a different colormap
# norm = plt.Normalize(7, 19)


# for i in range(7, 19):
#     current_array = np.array(curves_array[i])
#     colors = cmap(i)
#     ax.plot(
#         current_array[:, 0],
#         current_array[:, 3],
#         current_array[:, 1],
#         color=colors,
#     )
cmap_colors = plt.cm.viridis(np.linspace(0, 1, num=len(curves_array)))
cmap = LinearSegmentedColormap.from_list("CustomCmap", cmap_colors)

num_curves = 19 - 7

for i in range(8, 21):
    curve_color = cmap(i / num_curves - 1)
    current_array = np.array(curves_array[i])
    plt.plot(
        np.array(current_array[:, 0]),
        np.array(current_array[:, 2]) - 20 * i,
        color=curve_color,
    )
# ax.plot(
#     time_in_cycle,
#     cycle_number,
#     reference0,
# )
plt.grid()
plt.show()
