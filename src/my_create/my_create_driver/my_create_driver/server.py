import socket
import cv2
import numpy as np

from my_model import *
from abstacle_avoidance_algorithm import *

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind(('localhost', 8989))
serversocket.listen(1) # become a server socket, maximum 1 connections

print("Waiting connection...")
connection, address = serversocket.accept()
print(f"Connection from: {address}")

while True:
    # Read image size first
    length_bytes = connection.recv(10)

    # Wait client to connect again if it's closed
    if len(length_bytes) < 10:
        print("Client closed too early")
        connection.close()
        print("Waiting connection...")
        connection, address = serversocket.accept()
        print(f"Connection from: {address}")
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
    # camImage = camImage.reshape(512, 512)
    if camImage is not None:
        print(f"img size: {camImage.size}")

        # cv2.imshow('img', camImage)

        image = predict_image(None, camImage, False)
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



    # if cv2.waitKey(1) == ord('q'):
    #     break
# cv2.destroyAllWindows()
