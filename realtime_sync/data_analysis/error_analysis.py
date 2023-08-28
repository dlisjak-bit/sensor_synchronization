import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors as colors


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
fig.tight_layout(pad=3)
cax = fig.add_axes([0.92, 0.15, 0.02, 0.7])  # Position of the colorbar

for i in range(4):
    with open(f"alldata/error_logs{i}.csv", "r") as file:
        lines = file.readlines()

    # Initialize lists to store data
    cycle_number = []
    time_in_cycle = []
    relative_errors0 = []
    relative_errors1 = []
    speed_reduction = []
    current_cycle_number = 0

    # Process the data
    for line in lines:
        if line.strip() == "-":
            # New cycle detected
            current_cycle_number += 1
        else:
            # Parse the line and extract relevant values
            values = line.strip().split(",")
            speed_reduction.append(int(values[3]))
            time_in_cycle.append(float(values[0]))
            relative_errors0.append(float(values[1]))
            relative_errors1.append(float(values[2]))
            cycle_number.append(current_cycle_number)

    # Convert lists to NumPy arrays
    cycle_number = np.array(cycle_number)
    time_in_cycle = np.array(time_in_cycle)
    relative_errors0 = np.array(relative_errors0)
    relative_errors0_avg = np.array(relative_errors0)
    relative_errors1 = np.array(relative_errors1)
    relative_errors1_avg = np.array(relative_errors1)
    # if len(relative_errors1_avg) > 10:
    #     for idx, error in enumerate(relative_errors0_avg):
    #         if idx < 5:
    #             continue
    #         error = np.mean(relative_errors0[idx - 5 : idx + 1])
    #         relative_errors0_avg[idx] = error

    #     for idx, error in enumerate(relative_errors1_avg[5:]):
    #         if idx < 5:
    #             continue
    #         error = np.mean(relative_errors1[idx - 5 : idx + 1])
    #         relative_errors1_avg[idx] = error

    # Plot the heatmap
    # ax[i, 0].figure(figsize=(10, 6))  # Adjust the figure size as needed
    # ax[i, 0].set_ticks((np.linspace(0, max(cycle_number), 1)))
    # ax[i, 1].set_ticks((np.linspace(0, max(cycle_number), 1)))

    ax[i, 0].grid()
    ax[i, 1].grid()

    ax[i, 0].scatter(
        time_in_cycle,
        cycle_number,
        marker="|",
        c=relative_errors0_avg,
        cmap="plasma",
        vmin=-1,
        vmax=1,
    )
    ax[i, 0].set(
        xlabel="Time in Cycle",
        ylabel="Cycle Number",
        # yticks=np.arange(1, max(cycle_number), 5),
    )
    ax[i, 0].set_title(f"Board {i} Sensor 0")
    ax[i, 0].invert_yaxis()

    ax[i, 1].scatter(
        time_in_cycle,
        cycle_number,
        marker="|",
        cmap="plasma",
        c=relative_errors1_avg,
        vmin=-1,
        vmax=1,
    )
    ax[i, 1].set(
        xlabel="Time in Cycle",
        ylabel="Cycle Number",
        # yticks=np.arange(1, max(cycle_number), 5),
    )
    ax[i, 1].set_title(f"Board {i} Sensor 1")
    ax[i, 1].invert_yaxis()

    # speed_reduction = 0.5 / 100 * np.array(speed_reduction) + np.array(
    #     cycle_number
    # )
    # ax[i, 0].plot(
    #     time_in_cycle,
    #     speed_reduction,
    #     linewidth=0.5,
    # )
    # ax[i, 1].plot(
    #     time_in_cycle,
    #     speed_reduction,
    #     linewidth=0.5,
    # )


# Create a ScalarMappable for the colorbar
sm = plt.cm.ScalarMappable(cmap="plasma")
sm.set_array([])  # Set an empty array for now

norm = colors.Normalize(vmin=-1, vmax=1)
sm.set_norm(norm)
# Plot the colorbar
cbar = plt.colorbar(
    sm,
    cax=cax,
    # boundaries=np.linspace(-1, 1, 256),
)

cbar.set_label("Relative Errors")
cbar.set_ticks([-1, 0, 1])  # Set the tick positions
cbar.set_ticklabels(["-1", "0", "1"])  # Set the tick label


plt.savefig("data_analysis/analysis.png", dpi=400)
