import socket
from typing import List
from dto.robot import Move, Move
import math

server_ip = '192.168.137.3'  # Erstat med IP-adressen til din EV3
port = 5000
client_socket = socket.socket # Modified to not crash program on launch... Can't instantiate if no robot.

def connect():
    global client_socket
    client_socket = client_socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))

def prepare_command(move: Move):
    suck = move.suck
    speed = move.speed
    latch = 0
    lm = 0
    rm = 0
    if move.radians == 0:
        lm = (speed/2)
        rm = (speed/2)
    elif abs(move.radians) > 0.02 and abs(move.radians) < 0.03:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.5
        else:
            lm = (speed/2) * 0.5
            rm = (speed/2) * 1
    elif abs(move.radians) > 0.04 and abs(move.radians) < 0.06:z
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.25
        else:
            lm = (speed/2) * 0.25
            rm = (speed/2) * 1
    elif abs(move.radians) > 0.07 and abs(move.radians) < 0.08:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * -1
        else:
            lm = (speed/2) * -1
            rm = (speed/2) * 1
    elif abs(move.radians) > 0.09 and abs(move.radians) < 0.11:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * -1
        else:
            lm = (speed/2) * -1
            rm = (speed/2) * 1

    # Left motor speed, Right motor speed, fan on/off, latch open/close
    return lm, rm, 1 if suck else 0, 1 if latch else 0


def goal_command():
    move: Move = Move(speed=0, radians=0, suck=False, latch=True)
    client_socket.send("0 0 -1 1".encode())
    client_socket.recv(1024)

def send_command(move: Move):
    command = prepare_command(move)
    command_string = f"{command[0]} {command[1]} {command[2]} {command[3]}" #Left motor, Right motor, Fan, latch
    print(f'command string: {command_string}')
    client_socket.send(command_string.encode())
    client_socket.recv(1024)

