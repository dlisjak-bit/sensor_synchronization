import matplotlib.pyplot as plt
import numpy as np

# for i in range(4):
#     with open(f"alldata/adapted_reference{i}.csv", "w") as f:
#         f.close()
#     with open(f"alldata/error_logs{i}.csv", "w") as f:
#         f.close()
#     with open(f"alldata/interpolated_sensors{i}.csv", "w") as f:

#         f.close()
#     with open(f"alldata/sensor_data{i}.csv", "w") as f:
#         f.close()


fig, ax = plt.subplots(4, 2)
fig.set_figheight(8)
fig.set_figwidth(6)

for i in range(4):
    with open(f"alldata/error_logs{i}.csv", "r") as file:
        lines = file.readlines()

    # Initialize lists to store data
    cycle_number = []
    time_in_cycle = []
    relative_errors0 = []
    relative_errors1 = []
    current_cycle_number = 0

    # Process the data
    for line in lines:
        if line.strip() == "-":
            # New cycle detected
            current_cycle_number += 1
        else:
            # Parse the line and extract relevant values
            values = line.strip().split(",")
            time_in_cycle.append(float(values[0]))
            relative_errors0.append(float(values[1]))
            relative_errors1.append(float(values[2]))
            cycle_number.append(current_cycle_number)

    # Convert lists to NumPy arrays
    cycle_number = np.array(cycle_number)
    time_in_cycle = np.array(time_in_cycle)
    relative_errors0 = np.array(relative_errors0)
    relative_errors1 = np.array(relative_errors1)

    # Plot the heatmap
    # ax[i, 0].figure(figsize=(10, 6))  # Adjust the figure size as needed
    ax[i, 0].scatter(
        time_in_cycle,
        cycle_number,
        marker="s",
        c=relative_errors0,
        vmin=-1,
        vmax=1,
    )
    ax[i, 0].set(xlabel="Time in Cycle", ylabel="Cycle Number")
    ax[i, 0].set_title(f"Board {i} Sensor 0")
    ax[i, 0].invert_yaxis()

    ax[i, 1].scatter(
        time_in_cycle,
        cycle_number,
        marker="s",
        c=relative_errors1,
        vmin=-1,
        vmax=1,
    )
    ax[i, 1].set(xlabel="Time in Cycle", ylabel="Cycle Number")
    ax[i, 1].set_title(f"Board {i} Sensor 1")
    ax[i, 1].invert_yaxis()

plt.savefig("data_analysis/analysis.png")
