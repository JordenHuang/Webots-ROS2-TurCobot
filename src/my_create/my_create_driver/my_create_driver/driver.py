from pycreate2 import Create2
import time
import numpy as np
import cv2
import sys
import socket

from . import robot_util as ru
from . import direction

PORT = "/dev/ttyUSB0"

HIGH_VALUE = 10000
WIDTH = HIGH_VALUE
HEIGHT = HIGH_VALUE

# Open the default camera


def main():
    # '''
    # Greeting to see if this function is called
    print("Hello from my_create driver's main function")

    cam_left = cv2.VideoCapture(0)
    # cam_right = cv2.VideoCapture(0)
    if not cam_left.isOpened():
        print("Cannot open left camera")
        sys.exit()

    # if not cam_right.isOpened():
    #     print("Cannot open right camera")
    #     sys.exit()

    # if cam_left.isOpened() and cam_right.isOpened():
    #     print("Camera opened SUCCESSFULLY")

    # # Create a Create2.
    # my_create = Create2(PORT)
    # # start the bot
    # my_create.start()
    # # safe mode
    # my_create.safe()
    # time.sleep(1)

    # sensors = my_create.get_sensors()
    # battery_capacity = sensors.battery_capacity
    # battery_charge = sensors.battery_charge
    # print(f"Battery capacity: {battery_capacity}")
    # print(f"Battery charge: {battery_charge}")
    # print(f"Battery percentage: {battery_charge / battery_capacity * 100.0:.2f}% (approximatly)")
    # time.sleep(1)


    print("Try to connect to server...")
    clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientsocket.connect(('localhost', 8989))
    print("Connected")
    # clientsocket.settimeout(60)

    counter = 0
    while True:
        counter += 1
        time.sleep(0.2)
    # turn in place
        # ru.wheel_turn(my_create, "left", direction.Direction.FORWARD, 0.25)
        # ru.wheel_turn(my_create, "right", direction.Direction.FORWARD, 0.25)
        # time.sleep(2)
    # my_create.drive_direct(100,-100)  # inputs for motors are +/- 500 max
    # time.sleep(2)
    # my_create.drive_direct(-100,100)  # inputs for motors are +/- 500 max
    # time.sleep(2)
    # '''

        ret_left, frame_left = cam_left.read()
        # ret_right, frame_right = cam_right.read()
        if ret_left:
            cv2.imshow('Camera left', frame_left)

        # Display the captured frame
        if ret_left and counter % 10 == 0:
            counter = 0
            print(f"Img size: {str(frame_left.size)}", flush=True)
            # Encode the image to JPEG first
            encoded, buffer = cv2.imencode(".jpg", frame_left)
            buffer_bytes = buffer.tobytes()
            buffer_length = len(buffer_bytes)
            buffer_length_bytes = buffer_length.to_bytes(10, 'big')
            # clientsocket.send(str(frame_left.size).encode())
            clientsocket.sendall(buffer_length_bytes + buffer_bytes)
        # if ret_right:
        #     cv2.imshow('Camera right', frame_right)

            print("wait recv", flush=True)
            cmd_length_bytes = clientsocket.recv(10)
            cmd_length = int.from_bytes(cmd_length_bytes, 'big')
            print(f"cmd length: {cmd_length}", flush=True)
            cmd_bytes = b''
            while len(cmd_bytes) < cmd_length:
                packet = clientsocket.recv(cmd_length - len(cmd_bytes))
                if not packet:
                    break
                cmd_bytes += packet
            cmd = cmd_bytes.decode()
            print(f"Receive cmd: '{cmd}'", flush=True)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    cam_left.release()
    # cam_right.release()
    cv2.destroyAllWindows()
