from rich.console import Console
from rich.table import Table
import time
from threading import Thread

def send_command(a):
    print("a")

def thread_2():
    global user_input
    while True:
        user_input = input("Type input")

t1 = Thread(target = thread_2)
t1.start()

user_input = ""
robot_output = ["Reading reference...", "4.54645s"]
arduino_output_list = [[["warning", [3,4,5,5]], ["no warning", [0,0,0,0]]],[["no warning", [0,0,0,0]],["no warning", [0,0,1,0]]]]
while True:
    table = Table(title = "Monitoring output")
    table.add_column("Source")
    table.add_column("Board no.")
    table.add_column("Status")
    table.add_column("Data")

    table.add_row("Last command", user_input)
    table.add_row("Robot", "", robot_output[0], robot_output[1])
    for idx, arduino_board in enumerate(arduino_output_list):
        for sensor_idx, sensor in enumerate(arduino_board):
            table.add_row(f"Sensor{sensor_idx}", f"{idx}", f"{sensor[0]}", f"{sensor[1]}")

    console = Console()
    console.print(table, end="\r")
    #user_input = input('Type user input: ')
    # doing something with the input
    #send_command(user_input)
    time.sleep(0.2)
        
output = ""
output = f"Robot output "
for thing in robot_output:
    output += f"/ {thing}"
output += "\n"
for idx, arduino_board in enumerate(arduino_output_list):
    output += f"Arduino board {idx} "
    for thing in arduino_board:
        output += f"/ {thing}"
    output += "\n"
print(output)