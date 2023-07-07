import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure(figsize=(12, 12))

for i in range(3):
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

    # Create subplot for each board and sensor
    ax = fig.add_subplot(4, 2, 2 * i + 1, projection="3d")
    ax.scatter(
        time_in_cycle,
        cycle_number,
        relative_errors0,
        s=1,
        c=cycle_number,
        cmap="viridis",
    )
    ax.set_xlabel("Time in Cycle")
    ax.set_ylabel("Cycle Number")
    ax.invert_yaxis()
    ax.set_zlabel("Relative Error")
    ax.set_title(f"Board {i} Sensor 0")

    ax = fig.add_subplot(4, 2, 2 * i + 2, projection="3d")
    ax.scatter(
        time_in_cycle,
        cycle_number,
        relative_errors1,
        s=1,
        c=cycle_number,
        cmap="viridis",
    )
    ax.set_xlabel("Time in Cycle")
    ax.set_ylabel("Cycle Number")
    ax.invert_yaxis()
    ax.set_zlabel("Relative Error")
    ax.set_title(f"Board {i} Sensor 1")

# plt.tight_layout()
plt.show()
