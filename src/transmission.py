import socket
from typing import List
from dto import Inputs, Input
import math

server_ip = '192.168.52.77'  # Erstat med IP-adressen til din EV3
port = 5000
client_socket = socket.socket # Modified to not crash program on launch... Can't instantiate if no robot.

def connect():
    client_socket = client_socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))

def prepare_command(inputs: Inputs):
    commands = inputs.inputs
    suck = False
    speed = 0
    degress = 0
    for command in commands:
        if(command == Input.LEFT or command ==Input.RIGHT):
            if command.value < 0:
                degress += command.value - 2
            else:
                degress += command.value + 2
        if(command == Input.FORWARD or command == Input.BACKWARD):
            speed += command.value
        if(command == Input.SUCK):
            suck = command.value
    return 1 if suck else 0, speed*10, math.degrees(degress)

def send_command(commands: List[Input]):
    command = prepare_command(commands)
    command_string = f"{command[0]} {command[1]} {command[2]}"
    client_socket.send(command_string.encode())


