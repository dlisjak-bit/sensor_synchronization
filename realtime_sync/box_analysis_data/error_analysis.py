import matplotlib.pyplot as plt

NUM_ARDUINOS = 1
FILENAME = "error_logs"

for arduino_board_number in range(1, NUM_ARDUINOS + 1):
    file = f"{FILENAME}{arduino_board_number}.csv"
    data = [[], [], []]
    with open(file, "r") as f:
        for line in f:
            line = line.strip().split(",")
            for i in range(3):
                data[i].append(float(line[i]))
    # Split cycles
    cycles = []
    current_cycle = [[], [], []]
    prev_time = 0.0
    for i in range(len(data[0])):
        if data[0][i] < prev_time and data[0][i] < 0.1 and prev_time > 8:
            # New cycle:
            # print(data[0][i], prev_time)
            cycles.append(current_cycle)
            current_cycle = [[], [], []]
        for j in range(3):
            current_cycle[j].append(data[j][i])
        prev_time = data[0][i]
    print(len(cycles))
    for cycle in cycles:
        if len(cycle[0]) < 50:
            cycles.remove(cycle)
            # print(cycle)
    print(len(cycles))
    fig, axs = plt.subplots(len(cycles), figsize=(3.5 * 4, 3.5 * (30 / 4)))
    for i in range(len(cycles)):
        axs[i].scatter(cycles[i][0], cycles[i][1], label=f"relative error", s=0.8)
        axs[i].axhline(
            y=0.3, color="r", linestyle="-", linewidth=0.8, label="error threshold"
        )
        axs[i].set_ylim(-1.2, 1.2)
        axs[i].grid()
    # plt.legend()
    axs[0].legend()
    fig.tight_layout()
    plt.savefig("errors.png", dpi=150)
    # Calculate when error was above or below threshold
    # error_cycles = []
    # for idx, cycle in enumerate(cycles):
    #     error_cycle = [[], []]
    #     for i in range(len(cycle[0])):
    #         if cycle[1][i] > 0.3:
    #             error_cycle[0].append(cycle[0][i])
    #             error_cycle[1].append(0)
    #     error_cycles.append(error_cycle)
    # # Plot only above threshold
    # for i in range(29):
    #     axs[i].scatter(
    #         error_cycles[i][0],
    #         error_cycles[i][1],
    #         label=f"cycle{i}",
    #     )
    #     axs[i].set_ylim(-1, 1)
    # plt.legend()
    fig.tight_layout()
    plt.savefig("errors.png", dpi=150)
