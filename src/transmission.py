import socket
from typing import List
from dto.robot import Move, Move
import math

server_ip = '192.168.52.77'  # Erstat med IP-adressen til din EV3
port = 5000
client_socket = socket.socket # Modified to not crash program on launch... Can't instantiate if no robot.

def connect():
    client_socket = client_socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))

def prepare_command(move: Move):
    suck = move.suck
    speed = move.speed
    degress = math.degrees(move.radians)
    
    return 1 if suck else 0, speed*10, degress

def send_command(move: Move):
    command = prepare_command(move)
    command_string = f"{command[0]} {command[1]} {command[2]}"
    client_socket.send(command_string.encode())


