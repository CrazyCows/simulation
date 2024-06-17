import socket
from typing import List
from dto.robot import Move, Move
import math

server_ip = '192.168.88.77'  # Erstat med IP-adressen til din EV3
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
    elif abs(move.radians) == 0.025:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.75
        else:
            lm = (speed/2) * 0.75
            rm = (speed/2) * 1
    elif abs(move.radians) == 0.05:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.5
        else:
            lm = (speed/2) * 0.5
            rm = (speed/2) * 1
    elif abs(move.radians) == 0.075:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.25
        else:
            lm = (speed/2) * 0.25
            rm = (speed/2) * 1
    elif abs(move.radians) == 0.1:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * -1
        else:
            lm = (speed/2) * -1
            rm = (speed/2) * 1

    # Left motor speed, Right motor speed, fan on/off, latch open/close
    return lm, rm, 1 if suck else 0, 1 if latch else 0

def send_command(move: Move):
    command = prepare_command(move)
    command_string = f"{command[0]} {command[1]} {command[2]} {command[3]}" #Left motor, Right motor, Fan, latch
    client_socket.send(command_string.encode())


