#!/usr/bin/env python3

import cv2
import beholder
import numpy as np
import socket
import time
import dlib
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

# Variables for vid capture
reading_from_file = True  # Change this to capture from videofile instead of robot feed, for testing!
fps = 20
time_between_frames = 1000 // fps if reading_from_file else 1
if reading_from_file:
    cap = cv2.VideoCapture('Sign2.mkv')  # Change the string to choose which videofile to capture from

# Connection with raspberry to transmit commands
sock = socket.socket()
server_address = (IPadress, 1080)
if not reading_from_file:
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

if not reading_from_file:
    send_cmd(DEFAULT_CMD)
time.sleep(2)
flag = 1
key = 1
fn = 1
speed = 1550

# constants
trafficlight_detector = dlib.simple_object_detector("trafficlight_detector.svm")
h_min = np.array((0, 0, 215), np.uint8)
h_max = np.array((360, 255, 255), np.uint8)
base_shape = (720, 1280, 3)
base_traffic_shape = (85, 64, 3)
max_time_between_detection = 0.5  # in seconds
min_x = 90  # 90

# variables
static = True
stop = False
seeing_trafficlight = False
last_time_seen = 0
bound_top = -1
bound_bottom = -1
bound_left = -1
bound_right = -1
text = "ничего не горит"
color = (0, 0, 0)
prev_color = ""
blink_lock = False

# color pixel thresholds
red_thresh = 45000
yellow_thresh = 45000
green_thresh = 25000

pixel_thresholds = [red_thresh, yellow_thresh, green_thresh]


while cv2.waitKey(10) != ESCAPE:
    if reading_from_file:
        ret, frame = cap.read()
    else:
        status, frame = client.get_frame(0.25)  # read the sent frame
    if reading_from_file or status == beholder.Status.OK:
        # detection road
        if frame.shape != base_shape:
            frame = cv2.resize(frame, (base_shape[1], base_shape[0]))
        copy = frame.copy()
        img = cv2.resize(frame, (400, 300))
        binary = binarize(img, d=1)
        perspective = trans_perspective(binary, TRAP, RECT, SIZE)
        left, right = centre_mass(perspective, d=1)
        err = 0 - ((left + right) // 2 - 200)

        boxes = trafficlight_detector(copy)
        for box in boxes:
            (x, y, xb, yb) = [box.left(), box.top(), box.right(), box.bottom()]
            (bound_left, bound_top, bound_right, bound_bottom) = (max(x, 0), max(y, 0), max(0, xb), max(0, yb))
            print("found: ", x, y, xb, yb)
            cv2.rectangle(copy, (x, y), (xb, yb), (0, 0, 255), 2)
            width = bound_right - bound_left
            if width >= min_x * int(not static):  # If we are static, then we do not care about min X of the trafficlight (or, the distance to it)
                seeing_trafficlight = True
                last_time_seen = time.time()
        if len(boxes) == 0:
            if time.time() - last_time_seen > max_time_between_detection:  # Detecting if we completely lost the traffic light by the time between detection. Need a better way.
                seeing_trafficlight = False
        if seeing_trafficlight:
            traffic_cut_1 = frame[bound_top:bound_bottom, bound_left:bound_right]
            actual_shape = traffic_cut_1.shape
            traffic_cut_1 = cv2.resize(traffic_cut_1, (base_traffic_shape[1], base_traffic_shape[0]))
            hsv_traffic_cut = cv2.cvtColor(traffic_cut_1, cv2.COLOR_BGR2HSV_FULL)
            hsv_traffic_cut = cv2.medianBlur(hsv_traffic_cut, 5)

            mask_full = cv2.inRange(hsv_traffic_cut, h_min, h_max)

            y_cut = mask_full.shape[0] / 3

            cut_red = mask_full[0:int(y_cut)]
            cut_yellow = mask_full[int(y_cut):int(y_cut * 2)]
            cut_green = mask_full[int(y_cut * 2):-1]

            sum_red = np.sum(cut_red)
            sum_yellow = np.sum(cut_yellow)
            sum_green = np.sum(cut_green)
            pixel_sums = [sum_red, sum_yellow, sum_green]

            print("red: ", sum_red)
            print("yellow: ", sum_yellow)
            print("green: ", sum_green)

            red_state = sum_red >= red_thresh
            yellow_state = sum_yellow >= yellow_thresh
            green_state = sum_green >= green_thresh
            trafficlight_state = [red_state, yellow_state, green_state]
            print("state: ", trafficlight_state)


            text_pos = (bound_left, bound_bottom + 20)

            if red_state and yellow_state:
                stop = True
                prev_color = "redyellow"
                text = "красный+желтый"
                color = (0, 0, 255)
            elif red_state:
                stop = True
                prev_color = "red"
                text = "красный"
                color = (0, 0, 255)
            elif yellow_state:
                stop = True
                prev_color = "yellow"
                text = "желтый"
                color = (0, 255, 255)
            elif green_state and not blink_lock:
                stop = False
                prev_color = "green"
                text = "зеленый"
                color = (0, 255, 0)
            elif prev_color == "green":
                stop = False
                blink_lock = True
                text = "мигающий зеленый"
                color = (0, 255, 0)
            else:
                stop = False
                blink_lock = False

            print("act shape: ", actual_shape)
            copy = cv2.putText(copy, text, text_pos, cv2.FONT_HERSHEY_COMPLEX, 1, color)
            cv2.imshow("trafcut1", traffic_cut_1)
            cv2.imshow("full_mask", mask_full)

        cv2.imshow("Frame", copy)

        if abs(right - left) < 100:
            err = last
            print("LAST")

        angle = int(87 + KP * err + KD * (err - last))
        if angle < 70:
            angle = 70
        elif angle > 106:
            angle = 104

        last = err
        if stop:
            print("stopped")
        # send speed and angle to Eyecar
        if not reading_from_file:
            send_cmd('H00/' + str(speed * int(not (static or stop))) + '/' + str(angle)+"E")

        key = cv2.waitKey(time_between_frames)

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
cv2.destroyAllWindows()
if not reading_from_file:
    send_cmd(DEFAULT_CMD)  # Stop the car
    sock.close()
    client.stop()
