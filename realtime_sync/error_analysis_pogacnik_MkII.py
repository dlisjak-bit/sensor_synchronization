import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors as colors
import matplotlib.ticker as mtick
from matplotlib.patches import Rectangle
from matplotlib.colors import LinearSegmentedColormap

# IMPORTANT NOTES
# when doing analysis always throw away all data with min(t) and max(t)

gradient = LinearSegmentedColormap.from_list(
    "my_gradient",
    (
        # Edit this gradient at https://eltos.github.io/gradient/#0:001875-44:F0F0FF-50:FFFFFF-56:FFF0F0-100:A20008
        (0.000, (0.000, 0.094, 0.459)),
        (0.440, (0.941, 0.941, 1.000)),
        (0.500, (1.000, 1.000, 1.000)),
        (0.560, (1.000, 0.941, 0.941)),
        (1.000, (0.635, 0.000, 0.031)),
    ),
)


def getData(path, ch):
    # first obtain the error and reference data; error is a bit trickier due to fucky formatting
    error = []
    with open(f"{path}error_logs{ch}.csv", "r") as file:
        lines = file.readlines()
    for l in lines:
        if l.strip() == "-":
            pass
        else:
            # Parse the line and extract relevant values
            values = [float(i) for i in l.strip().split(",")]
            error.append(values)
    error = np.array(error)
    print(error.shape)
    print(np.min(error[:, 0]), np.max(error[:, 0]))
    error = error[np.where(error[:, 0] != max(error[:, 0]))[0], :]
    # When first column of data is equal to min(first column of data), throw
    # away the whole row (this is a bug in the data collection)
    error = error[np.where(error[:, 0] != min(error[:, 0]))[0], :]
    print(error.shape)
    reference = np.genfromtxt(f"{path}adapted_reference{ch}.csv", delimiter=",")
    # get max time
    max_time = np.max(error[:, 0])
    # normalize the execution time to make fitting easier
    error[:, 0] = error[:, 0] / np.max(error[:, 0])
    reference[:, 0] = reference[:, 0] / np.max(reference[:, 0])

    # unwrap the twisted time
    error_unwrappedTime = np.unwrap(error[:, 0], period=1)
    reference_unwrappedTime = np.unwrap(reference[:, 0], period=1)

    error[:, 0] = error_unwrappedTime
    reference[:, 0] = reference_unwrappedTime

    # remove duplicates in the error time becuase fuck this shit...
    error_c = np.copy(error)
    while 1:
        error_timeDiff = np.diff(error[:, 0])
        error_repeatPoints = np.where(error_timeDiff < 0)[0]
        if len(error_repeatPoints):
            # breakpoint()
            x = (
                np.argmax(
                    error[error_repeatPoints[0] : :, 0]
                    > error[error_repeatPoints[0], 0]
                )
                + error_repeatPoints[0]
                - 1
            )
            # print(x)
            # error = error[error_repeatPoints[0]:x,:]
            error = np.append(
                error[0 : error_repeatPoints[0], :], error[x::, :], axis=0
            )

        else:
            break
    # remove sequential duplicates
    error = error[np.unique(error[:, 0], axis=0, return_index=True)[1], :]

    # match samples in reference with samples in error file
    d = np.clip(
        np.searchsorted(reference[:, 0], error[:, 0]), 0, reference.shape[0] - 1
    )

    # add reference samples to the error array
    error = np.append(error, reference[d, 1:3], axis=1)

    # add absolute error to the array
    error = np.append(error, np.multiply(error[:, 1:3], error[:, 4:6]), axis=1)
    return error, max_time


def processData(path, maxCycles=1e6):
    # fig, ax = plt.subplots(4, 2, constrained_layout=True)
    fig, ax = plt.subplots(4, 2)
    fig.set_figheight(8)
    fig.set_figwidth(6)
    fig.tight_layout(pad=3)

    fig.canvas.manager.set_window_title(path.split("/")[-1])
    fig.tight_layout()

    for i in range(4):
        data, max_time = getData(path, i)
        # breakpoint()

        # remove first line
        data = data[np.where(data[:, 0] > 1)[0], :]

        # remove all lines where the value of the data[:, 0] == 0
        # data = data[np.where((data[:, 0] // 1) != 0.0)[0], :]

        # clip the number of lines
        data = data[
            np.where(data[:, 0] < (min(maxCycles, int(np.max(data[:, 0])))))[0], :
        ]

        selectWhatToPlot = 0
        # When first column of data is equal to max(first column of data), throw
        # away the whole row (this is a bug in the data collection)

        dataToPlot = data[:, 1:3]
        dataRange = [-1, 1]
        scaleLabel = "Relative sensor reading deviation"
        if selectWhatToPlot == 1:
            dataToPlot = data[:, 4:6]
            dataRange = [-500, 500]
            scaleLabel = "Absolute sensor readings"
        elif selectWhatToPlot == 2:
            dataToPlot = data[:, 6:8]
            dataRange = [-500, 500]
            scaleLabel = "Sensor reading deviation from reference"

        # plot graphs with error data
        for j in range(2):
            # ax[i, j].grid()
            # breakpoint()
            # ax[i, j].set_facecolor((.9,.9,.9))#, hatch="\\")
            ax[i, j].add_patch(
                Rectangle(
                    (0, 0.6),
                    max_time,
                    -0.2 + int(np.max(data[:, 0])),
                    fill=True,
                    color=(0.9, 0.9, 0.9),
                )
            )
            ax[i, j].add_patch(
                Rectangle(
                    (0, 0.6),
                    max_time,
                    -0.2 + int(np.max(data[:, 0])),
                    fill=False,
                    color=(0.5, 0.5, 0.5),
                    hatch="\\\\\\",
                )
            )
            ax[i, j].scatter(
                (data[:, 0] % 1.0) * max_time,
                data[:, 0] // 1,
                marker="|",
                s=25,  # marker size
                c=dataToPlot[:, j],
                cmap=gradient,
                vmin=dataRange[0],
                vmax=dataRange[1],
                linewidths=2,
            )
            if j == 0:
                ax[i, j].set(  # xlabel="Percentage of Cycle",
                    ylabel="Cycle Number",
                    # yticks=np.arange(1, max(cycle_number), 5),
                )
            ax[i, j].set_xticks(np.arange(0, max_time + 1, 5))
            ax[i, j].grid()
            ax[i, j].set_title(f"Sensor {i}.{j}", fontsize=10, y=0.97)
            ax[i, j].invert_yaxis()
            # ax[i, j].xaxis.set_major_formatter(mtick.PercentFormatter())

    # Create a ScalarMappable for the colorbar
    cax = fig.add_axes([0.92, 0.15, 0.02, 0.7])  # Position of the colorbar
    sm = plt.cm.ScalarMappable(cmap=gradient)
    sm.set_array([])  # Set an empty array for now

    norm = colors.Normalize(vmin=dataRange[0], vmax=dataRange[1])
    sm.set_norm(norm)
    # Plot the colorbar
    cbar = plt.colorbar(
        sm,
        cax=cax,
        pad=1,
        # boundaries=np.linspace(-1, 1, 256),
    )

    cbar.set_label(scaleLabel)
    cbar.set_ticks([dataRange[0], 0, dataRange[1]])  # Set the tick positions
    cbar.ax.yaxis.set_ticks_position("left")
    # cbar.set_ticklabels(["-1", "0", "1"])  # Set the tick label

    # plt.subplot_tool()

    plt.subplots_adjust(right=0.85, wspace=0.15)

    fig.canvas.manager.set_window_title(f"{path.split('/')[-2]}")
    plt.savefig(f"{path}analysis_{path.split('/')[-2]}.png", dpi=400)
    plt.show(block=False)


if __name__ == "__main__":
    paths = [
        # "../alldata/2_robot_experiment/",
        #  "../alldata/box_experiment/",
        #  "../alldata/conveyor_experiment/",
        #  "../alldata/conveyor_experiment_2/",
        "alldata_attempt2/conveyor_duo1/"
    ]
    cycles = [1e6, 1e6, 1e6, 1e6, 1e6]

    # for path in paths[0:1]:
    for i, path in enumerate(paths):
        processData(path, cycles[i])
