import socket
import time
import os
import pigpio
from EncoderClass import Encoder


def setup_gpio():
    os.system("sudo pigpiod")  # Launching GPIO library
    time.sleep(1)
    ESC = 17
    STEER = 18
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(ESC, 0)
    pi.set_servo_pulsewidth(STEER, 0)
    time.sleep(1)
    # pi.set_servo_pulsewidth(ESC, 1500)
    # time.sleep(1)
    return pi, ESC, STEER


def setup_socket(port):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('', port)
    sock.bind(server_address)
    sock.listen(1)
    return sock


def control(pi, ESC, speed, STEER, angle):
    pi.set_servo_pulsewidth(ESC, speed)
    pi.set_servo_pulsewidth(STEER, int(16.6666666*angle))


def main():
    port = 1080
    sock = setup_socket(port)

    pi, ESC, STEER = setup_gpio()
    time.sleep(1)

    control(pi, ESC, 1500, STEER, 90)  # for calibration the ESC
    time.sleep(2)
    # encoder = Encoder()
    # encoder.start()
    # encoder.set_motor_control(ESC, pi, 300)

    while (True):  #
        control(pi, ESC, 1500, STEER, 90)
        print("Waiting for a connection")
        connection, client_address = sock.accept()
        print("ConnectionEstablished")
        # connection.settimeout(2)
        header = 0
        message = ""
        while (True):
            data = connection.recv(1)
            if data:
                dataStr = data.decode("utf-8")
                if dataStr == "H":
                    message = ""
                    header = 1
                elif header == 1 and dataStr != "E":
                    message += dataStr
                elif header == 1 and dataStr == "E":
                    header = 0
                    data_arr = message.split("/")
                    data_arr = list(map(int, data_arr))
                    speed = data_arr[1]
                    angle = data_arr[2]
                    print(speed, angle)

                    control(pi, ESC, speed, STEER, angle)
            else:
                print("Connection is broken")
                break


if __name__ == '__main__':
    main()
