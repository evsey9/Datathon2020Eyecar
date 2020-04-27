#!/usr/bin/env python3

import cv2
import beholder
import numpy as np
import socket
import time

from func import *

def send_cmd(cmd):
        message = cmd.encode()
        sock.sendall(message)


DEFAULT_CMD = 'H11/1500/90E'

KP = 0.32  #0.22
KD = 0.17
last = 0

SIZE = (400, 300)

RECT = np.float32([[0, 299],
                   [399, 299],
                   [399, 0],
                   [0, 0]])

TRAP = np.float32([[0, 299],
                   [399, 299],
                   [320, 200],
                   [80, 200]])

timeout = 0
l = 1
r = 0

povor = 0
totl = 1
pid = 0

ESCAPE = 27
SPASE = 32

i=1
j = 0

IPadress = "192.168.1.104"

# Connection with raspberry to transmit commands
sock = socket.socket()
server_address = (IPadress, 1080)
sock.connect(server_address)
print("Connection Established")

# Request a video stream from Eyecar
client = beholder.Client(zmq_host=IPadress,
                         # zmq_host="192.168.1.145",
                         zmq_port=12345,
                         rtp_host="192.168.1.208",
                         # rtp_host="10.205.1.185",
                         rtp_port=5000,
                         rtcp_port=5001,
                         device="/dev/video0",
                         # width=1920,
                         # height=1080,
                         width=1280,
                         height=720,
                         # width=640,
                         # height=480,
                         framerate=30,
                         encoding=beholder.Encoding.MJPEG,  #MJPEG,    #H264
                         limit=20)

client.start()

#
cv2.namedWindow("Frame")

send_cmd(DEFAULT_CMD)
time.sleep(2)
flag = 1
key = 1
fn = 1
speed = 1548

while cv2.waitKey(10) != ESCAPE:
    status, frame = client.get_frame(0.25)  # read the sent frame
    if status == beholder.Status.OK:
        cv2.imshow("Frame", frame)

        # detection road
        img = cv2.resize(frame, (400, 300))
        binary = binarize(img, d=1)
        perspective = trans_perspective(binary, TRAP, RECT, SIZE)
        left, right = centre_mass(perspective, d=1)
        err = 0 - ((left + right) // 2 - 200)

        if abs(right - left) < 100:
            err = last
            print("LAST")

        angle = int(87 + KP * err + KD * (err - last))
        if angle < 70:
            angle = 70
        elif angle > 106:
            angle = 104

        last = err

        # send speed and angle to Eyecar
        send_cmd('H00/' + str(speed) + '/' + str(angle)+"E")

        key = cv2.waitKey(1)

    elif status == beholder.Status.EOS:
        print("End of stream")
        break
    elif status == beholder.Status.Error:
        print("Error")
        break
    elif status == beholder.Status.Timeout:
        # Do nothing
        pass

# Completion of work
send_cmd(DEFAULT_CMD)  # Stop the car
sock.close()
cv2.destroyAllWindows()
client.stop()
