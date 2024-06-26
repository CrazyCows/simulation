#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import I2CDevice, UARTDevice
from pybricks.tools import wait
#import atexit
import signal
import sys

# ps aux | grep brickrun
# sudo kill xxxx

#from ev3_controller import Ev3Controller
import connection



# Initialize the EV3 brick
ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

arduino = UARTDevice( Port.S4, 9600 )
# Opens server on port 5000 by default
server_socket = connection.create_server_socket()
client_socket = connection.accept_connection(server_socket)

def close_all_sockets():
    try:
        server_socket.close()
        client_socket.close()
        #print(f"Closed server socket: {server_socket.getsockname()}")
    except Exception as e:
        print("error")

def signal_handler(sig, frame):
    print("Signal received, shutting down...")
    close_all_sockets(server_socket, client_socket)
    sys.exit(0)

# Register the signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# Register atexit to ensure sockets are closed on normal exit
#atexit.register(close_server_socket, server_socket)


def split_command(command: str):
    split_command_array = command.split()
    try:
        lm_speed = float(split_command_array[0])
        rm_speed = float(split_command_array[1])
        suck = int(split_command_array[2])
        latch = int(split_command_array[3])
        print(lm_speed, rm_speed, suck)
        return lm_speed, rm_speed, suck, latch
    except Exception as e:
        print("Error in split_command:", e)

def robot_run(lm_speed, rm_speed):
    # Motor max = 800
    max_speed = 800
    left_motor.run(max_speed*lm_speed)
    right_motor.run(max_speed*rm_speed)

def robot_stop():
    left_motor.stop()
    right_motor.stop()

def fan_start():
    try:
        arduino.write(b'f')
    except Exception as e:
        print("fan_start",e)

def fan_slow():
    try:
        arduino.write(b'F')
    except Exception as e:
        print("fan_slow",e)

def fan_off():
    try:
        arduino.write(b'S')
    except Exception as e:
        print("fan_off",e)

def fan_on():
    try:
        arduino.write(b's')
    except Exception as e:
        print("fan_on",e)

def latch_open():
    try:
        arduino.write(b'l')
    except Exception as e:
        print("latch_open",e)

def latch_close():
    try:
        arduino.write(b'L')
    except Exception as e:
        print("latch_close",e)

def latch_calibrate_one():
    try:
        arduino.write(b'c')
    except Exception as e:
        print("latch_close",e)

def latch_calibrate_two():
    try:
        arduino.write(b'C')
    except Exception as e:
        print("latch_close",e)

def fan_test():
    try:
        #arduino.write(b't')
        print("Opening latch")
        latch_open()
        wait(5000)
        print("Closing latch")
        latch_close()
    except Exception as e:
        print("fan_start",e)
que = []

try:
    is_sucking = 0
    is_latch_open = 0
    while True:
        #try:
            # data = client_socket.recv(1024).decode()
            que.append(client_socket.recv(1024).decode())
            data = que[0]
            que.pop(0)
            print("Command String recived:", data)

            if data == "c":
                latch_calibrate_one()
                continue
            if data == "cc":
                latch_calibrate_two()
                continue
            if data == "t":
                fan_test()
                continue
            if data == "t":
                fan_test()
                continue
            if data == "exit":
                break
            if data == "status":
                print(left_motor.control.limits())
                print(right_motor.control.limits())
                continue
            lm_speed, rm_speed, suck, latch = split_command(data)
            if lm_speed != 0 or rm_speed != 0:
                robot_run(lm_speed, rm_speed)
                print("Robt_run:\nLeft motor speed:",lm_speed,"\nRight motor speed:",rm_speed)
            elif lm_speed == 0 and rm_speed == 0:
                robot_stop()
                print("Robot_stop")
            if is_sucking != suck:
                print("Hallos")
                if is_sucking == -1:
                    fan_on()
                    print("Fan switch has been turned on. Fan is now able to run")
                is_sucking = suck
                if suck == 1:
                    print("Fan started")
                    fan_start()
                elif suck == 0:
                    fan_slow()
                    print("Fan slowed down")
                elif suck == -1:
                    fan_off()
                    print("Fan switched off completely.")
            if is_latch_open != latch:
                is_latch_open = latch
                if latch == 1:
                    latch_open()
                    print("Latch open")
                elif latch == 0:
                    latch_close()
                    print("Latch close")
            client_socket.send("1")
    """
            except Exception as e:
                print("Something went wrong when writing. While loop")
                print(e)
                print("Shutting down")
                close_all_sockets()
                print("client and server sockets closed")
    """

except Exception as e:
    print("Something went wrong when writing")
    print(e)
finally:
    print("Shutting down")
    fan_slow()
    close_all_sockets()
    print("client and server sockets closed")
