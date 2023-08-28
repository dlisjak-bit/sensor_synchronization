import numpy as np
import matplotlib.pyplot as plt

with open("alldata/motion_demo/robot_reference.csv", "r") as file:
    lines = file.readlines()

joint_positions = []
joint_velocities = []
time = []
for line in lines[2:]:
    values = line.strip().split(",")
    joint_positions.append([float(values[i]) for i in range(1, 7)])
    joint_velocities.append([float(values[i]) for i in range(7, 13)])
    time.append(float(values[0]))

time = np.array(time) - np.min(time)
# time = time / np.max(time) * 100
for i in range(6):
    plt.plot(time, np.array(joint_positions)[:, i], label=f"Joint {i}")

plt.legend()
plt.savefig("alldata/motion_demo/robot_reference.png")

plt.clf()

for i in range(6):
    plt.plot(time, np.array(joint_velocities)[:, i], label=f"Joint {i}")
plt.legend()
plt.savefig("alldata/motion_demo/robot_reference_velocities.png")

# plt.show()

# Select point:
# point : -0.03	-1.61	-1.39	-1.68	1.62	1.83	0.21	0.63	-0.01	-0.72	-0.04	0.2	1	-8732.04	-2832.1	21507.65	-76.64	220.66	37.88	1	769

point = [
    -0.03,
    -1.61,
    -1.39,
    -1.68,
    1.62,
    1.83,
    0.21,
    0.63,
    -0.01,
    -0.72,
    -0.04,
    0.2,
]


def get_euclidean(err):
    d_euclidean = np.sqrt(np.sum(np.square(err), axis=0))
    p_min_euclidean = np.where(d_euclidean == np.amin(d_euclidean))
    return d_euclidean, p_min_euclidean


def read_robot_reference(file):
    """Read robot reference file."""

    data = []
    with open(file, "r") as f:
        f.readline()  # throw away the header line
        for line in f:
            data.append(
                [float(x) for x in line.strip().split(",")]
            )  # read csv
    data = np.array(data).transpose()  # transpose matrix
    # separate data:
    return (
        data[0] - data[0][0],
        data[1:13],
        data[14:20],
        data[-2],
        data[-1],
    )  # time points array ([time]-[start time]), 12 arrays (q0 do qd5) , array tsf, array do


tref, ref, forces, pct100, do100 = read_robot_reference(
    "alldata/motion_demo/robot_reference.csv"
)
print(ref)


def get_normalized_t(
    ref, tref, pt, previous_t, method="euclidean", spread=2, subdivisions=10
):
    """Use given method (euclidean/manhattan) to find nearest point in reference sample, interpolate nearby points to find precise reference time"""

    if method == "euclidean":
        get_distance = get_euclidean
    else:
        return -1
    # Search only ref with max 0.1s ahead of previous_t
    tref_np = np.asarray(tref)
    t_end = previous_t + 3
    t_start = previous_t
    idx_end = min((np.abs(tref_np - t_end)).argmin(), len(tref))
    idx_start = (np.abs(tref_np - t_start)).argmin()
    ref_interp = ref[:, idx_start : idx_end + 1]
    d, p = get_distance(ref_interp - pt)
    idxl = max(0, p[0][0] + idx_start)
    idxh = min(len(ref[0]), p[0][0] + idx_start + spread + 2)
    didx = idxh - idxl

    # Allocate array to divide time into more sample points
    samplepts = np.linspace(tref[idxl], tref[idxh - 1], didx * subdivisions)

    # Interpolate the data between reference time points and fill samplepts array
    subdivided = interp_nd(samplepts, tref[idxl:idxh], ref[:, idxl:idxh])

    # Find a more precise reference time estimate
    d, p = get_distance(subdivided - pt)
    return samplepts[p[0][0]]


datapt = np.array(point).transpose()
datapt = datapt.reshape(12, 1)

# clear previous plot
plt.clf()
d, p = (get_euclidean(ref[:6, :] - datapt[:6]), tref)
print(d)
plt.plot(time, d[0][1:], label="Euclidean distance - position")
d, p = (get_euclidean(ref[6:, :] - datapt[6:]), tref)
plt.plot(time, d[0][1:], label="Euclidean distance - speed")
d, p = (get_euclidean(ref[:, :] - datapt[:]), tref)
plt.plot(
    time, d[0][1:] + d[0][1:], label="Euclidean distance - position + speed"
)
plt.legend()

plt.savefig("alldata/motion_demo/robot_reference_euclidean.png")

# clear previous plot
plt.clf()
