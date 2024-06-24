import socket
from typing import List
from dto.robot import Move, Move
import math
import signal
import atexit
import sys
import numpy as np

server_ip = '192.168.137.241'  # Erstat med IP-adressen til din EV3
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

    radians = move.radians
    suck = move.suck
    speed = move.speed / 2 # WE PUT IT DOWN, TOO FAST FOR NOW!

    latch = 0
    lm = 0
    rm = 0



    abs_radians = abs(radians)
    if abs_radians <= 0.1:
        rm = speed
        lm = speed
    elif 0.1 < abs_radians <= 0.4:
        if radians < 0:
            rm = 0.1
            lm = -0.1
        else:
            rm = -0.1
            lm = 0.1
    elif 0.4 < abs_radians <= 0.8:
        if radians < 0:
            rm = 0.5
            lm = -0.5
        else:
            rm = -0.5
            lm = 0.5
    elif 0.8 < abs_radians <= 1:
        if radians < 0:
            rm = 0.7
            lm = -0.7
        else:
            rm = -0.7
            lm = 0.7
    elif 1 < abs_radians <= 1.2:
        if radians < 0:
            rm = 0.9
            lm = -0.9
        else:
            rm = -0.9
            lm = 0.9
    else:
        if radians < 0:
            rm = speed
            lm = -speed
        else:
            rm = -speed
            lm = speed

    # Left motor speed, Right motor speed, fan on/off, latch open/close
    return lm, rm, 1 if suck else 0, 1 if latch else 0




def goal_command():
    move: Move = Move(speed=0, radians=0, suck=False, latch=True)
    send_command(move)

def send_command(move: Move):
    command = prepare_command(move)
    #print("command string:", command)
    command_string = f"{command[0]} {command[1]} {command[2]} {command[3]}" #Left motor, Right motor, Fan, latch
    # command_string = f"0 0 {command[2]} {command[3]}"  # Left motor, Right motor, Fan, latch
    #print(f'command string: {command_string}')
    client_socket.send(command_string.encode())
    client_socket.recv(1024)


def signal_handler(sig, frame):
    #print("Signal received, shutting down...")
    disconnect()
    sys.exit(0)

def exit_functions():

    # Register the signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Register atexit to ensure sockets are closed on normal exit
    atexit.register(disconnect)

