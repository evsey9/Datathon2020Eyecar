import cv2
import numpy as np


def binarize(img, d=0):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    binary_h = cv2.inRange(hls, (0, 0, 30), (255, 255, 255))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_g = cv2.inRange(gray, 150, 255) #130

    binary = cv2.bitwise_and(binary_g, binary_h)

    if d:
        cv2.imshow('hls', hls)
        cv2.imshow('hlsRange', binary_h)
        cv2.imshow('grayRange', binary_g)
        cv2.imshow('gray', gray)
        cv2.imshow('bin', binary)

    # return binary
    return binary_g


def trans_perspective(binary, trap, rect, size, d=0):
    matrix_trans = cv2.getPerspectiveTransform(trap, rect)
    perspective = cv2.warpPerspective(binary, matrix_trans, size, flags=cv2.INTER_LINEAR)
    if d:
        cv2.imshow('perspective', perspective)
    return perspective


def find_left_right(perspective, d=0):
    hist = np.sum(perspective[perspective.shape[0] // 3:, :], axis=0)
    mid = hist.shape[0] // 2
    left = np.argmax(hist[:mid])
    right = np.argmax(hist[mid:]) + mid
    if left <= 10 and right - mid <= 10:
        right = 399

    if d:
        cv2.line(perspective, (left, 0), (left, 300), 50, 2)
        cv2.line(perspective, (right, 0), (right, 300), 50, 2)
        cv2.line(perspective, ((left + right) // 2, 0), ((left + right) // 2, 300), 110, 3)
        cv2.imshow('lines', perspective)

    return left, right


def centre_mass(perspective, d=0):
    hist = np.sum(perspective, axis=0)
    if d:
        cv2.imshow("Perspektiv2in",perspective)

    mid = hist.shape[0] // 2
    i = 0
    centre = 0
    sum_mass = 0
    while (i <= mid):
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass>0:
        mid_mass_left = centre / sum_mass
    else:
        mid_mass_left = mid-1

    centre = 0
    sum_mass = 0
    i = mid
    while (i < hist.shape[0]):
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass>0:
        mid_mass_right = centre / sum_mass
    else:
        mid_mass_right = mid+1

    # print(mid_mass_left)
    # print(mid_mass_right)
    mid_mass_left = int(mid_mass_left)
    mid_mass_right = int(mid_mass_right)
    if d:
        cv2.line(perspective, (mid_mass_left, 0), (mid_mass_left, 300), 50, 2)
        cv2.line(perspective, (mid_mass_right, 0), (mid_mass_right, 300), 50, 2)
        # cv2.line(perspective, ((mid_mass_right + mid_mass_left) // 2, 0), ((mid_mass_right + mid_mass_left) // 2, 300), 110, 3)
        cv2.imshow('CentrMass', perspective)

    return mid_mass_left, mid_mass_right
