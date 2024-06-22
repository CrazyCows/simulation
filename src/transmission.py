import socket
from typing import List
from dto.robot import Move, Move
import math
import signal
import atexit
import sys

server_ip = '192.168.137.56'  # Erstat med IP-adressen til din EV3
port = 5000
client_socket = socket.socket # Modified to not crash program on launch... Can't instantiate if no robot.

def connect():
    global client_socket
    client_socket = client_socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))
    print("I connected to the EV3!")


def disconnect():
    if client_socket:
        send_exit_command()
        client_socket.close()
        print("Disconnected from the EV3.")

def send_exit_command():
    if client_socket:
        try:
            client_socket.send('exit'.encode())
            client_socket.recv(1024)
            print("Sent 'exit' command to the EV3.")
        except Exception as e:
            print(f"Error sending 'exit' command: {e}")


def prepare_command(move: Move):
    suck = move.suck
    speed = move.speed
    latch = 0
    lm = 0
    rm = 0
    if move.radians < 1:
        lm = (speed/2)
        rm = (speed/2)
    elif 0.04 < abs(move.radians) < 0.1:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.5
        else:
            lm = (speed/2) * 0.5
            rm = (speed/2) * 1
    elif 0.08 < abs(move.radians) < 0.3:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * 0.25
        else:
            lm = (speed/2) * 0.25
            rm = (speed/2) * 1
    elif 0.7 < abs(move.radians) < 0.7:
        if move.radians < 0:
            lm = (speed/2) * 1
            rm = (speed/2) * -1
        else:
            lm = (speed/2) * -1
            rm = (speed/2) * 1
    elif abs(move.radians) > 1.56:
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
    send_command(move)

def send_command(move: Move):
    command = prepare_command(move)
    #command_string = f"{command[0]} {command[1]} {command[2]} {command[3]}" #Left motor, Right motor, Fan, latch
    command_string = f"0 0 {command[2]} {command[3]}"  # Left motor, Right motor, Fan, latch
    print(f'command string: {command_string}')
    client_socket.send(command_string.encode())
    client_socket.recv(1024)


def signal_handler(sig, frame):
    print("Signal received, shutting down...")
    disconnect()
    sys.exit(0)

# Register the signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Register atexit to ensure sockets are closed on normal exit
atexit.register(disconnect)

