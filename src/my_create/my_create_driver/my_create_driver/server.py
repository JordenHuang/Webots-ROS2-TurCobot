'''
TODO:
    cv2.imshow() should be in main thread
'''
import socket
import cv2
import numpy as np
import threading

from my_model import *
from abstacle_avoidance_algorithm import *

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind(('localhost', 8989))
# serversocket.bind(('172.20.10.13', 8989))
serversocket.listen(1) # become a server socket, maximum 1 connections

print("Waiting connection...")
connection, address = serversocket.accept()
print(f"Connection from: {address}")

received_image = None
masked_image = None
close_window = False
stop_thread = False

def display_image():
    global stop_thread, close_window, received_image, masked_image
    while stop_thread == False:
        if close_window == True:
            cv2.destroyAllWindows()
        else:
            if received_image is not None:
                cv2.imshow("Image", received_image)
            if masked_image is not None:
                cv2.imshow("Mask image", masked_image)
            cv2.waitKey(1)
    cv2.destroyAllWindows()

a = threading.Thread(target=display_image, daemon=True)


try:
    a.start()
    while True:
        # Read image size first
        length_bytes = connection.recv(10)

        # Wait client to connect again if it's closed
        if len(length_bytes) < 10:
            print("Connection closed")
            connection.close()
            close_window = True
            print("Waiting connection...")
            connection, address = serversocket.accept()
            print(f"Connection from: {address}")
            close_window = False
            continue

        length = int.from_bytes(length_bytes, 'big')
        print(f"Get length: {length}")
        buf = b''
        while len(buf) < length:
            packet = connection.recv(length - len(buf))
            if not packet:
                break
            buf += packet

        print(f"Buffer len: {len(buf)}")
        buf = np.frombuffer(buf, dtype='uint8')
        print(f"Buffer: {buf}")


        camImage = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        received_image = camImage
        # camImage = camImage.reshape(512, 512)
        if camImage is not None:
            print(f"img size: {camImage.size}")

            image = predict_image(None, camImage, False)
            masked_image = image
            row_line_ok = 180 # 128 + 32
            cpDot, middleDots = getControlPoint(image, row_line_ok)
            print(f"cp dot is: {cpDot}")

            # Some parameters
            row_line_ok = 180 # 128 + 32
            row_line_back = 200 # 230
            range_accept = 10 #80
            image_middle_col = int(image.shape[1]/2)

            cmd = ""
            if cpDot:
                if cpDot[1][1] > row_line_back:
                    cmd = "back"
                # should go left more
                elif cpDot[1][0] < (image_middle_col-range_accept):
                    cmd = "left"
                # should go right more
                elif cpDot[1][0] > (image_middle_col+range_accept):
                    cmd = "right"
                else:
                    cmd = "front"
            else:
                cmd = "back"
            print("cmd to send:", cmd)
            print(f"cmd length: {len(cmd)}")
            cmd_length = len(cmd).to_bytes(10, 'big')
            connection.sendall(cmd_length + cmd.encode())
except KeyboardInterrupt:
    print("KeyboardInterrupted, close server")
    # if cv2.waitKey(1) == ord('q'):
    #     break
    received_image = None
    masked_image = None
    stop_thread = True
    a.join()
