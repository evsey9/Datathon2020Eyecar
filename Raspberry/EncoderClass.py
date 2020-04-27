import collections
import time
import RPi.GPIO as GPIO

import threading

mutex = threading.Lock()


class Encoder:
    def __init__(self, encoderGPIO=27, dequeSize=10):
        self.encoderGPIO = encoderGPIO
        self.motorGPIO = None
        self.pi = None   # pi - obgect from pigpio library, for pi.set_servo_pulsewidth(STEER, 0)
        self.speedControl = False
        self.stopped = False
        self.timeDeque = collections.deque()
        self.dequeSize = dequeSize
        self.lastEventTime = 0.0
        self.lastError = 0.0
        self.sumTime = 0.0
        self.averageTime = 0.0
        self.needTime = 0.0
        self.speed = 1548
        self.lastError = 0.0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoderGPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print("Encoder_Init")

    def set_motor_control(self, motor_pin, pi, need_time=0.0):
        mutex.acquire()
        self.pi = pi
        self.motorGPIO = motor_pin
        self.needTime = need_time
        mutex.release()

    def start_speed_control(self):
        mutex.acquire()
        if (self.motorGPIO == None) or (self.pi == None):
            print ("No set motor_pin and pi")
            mutex.release()
            return
        self.speedControl = True
        if self.stopped is True:
            self.stopped = True
            self.start()
        mutex.release()

    def stop_speed_control(self):
        mutex.acquire()
        self.speedControl = False
        self.pi.set_servo_pulsewidth(self.motorGPIO, 1500)
        mutex.release()
        print("SpeedControl stopped")

    def update(self, pin):  # pin - to receive a parameter from an interrupt handler
        delta_time = (time.monotonic()-self.lastEventTime)*1000
        self.lastEventTime = time.monotonic()

        mutex.acquire()
        if (self.speedControl is True) and (self.needTime > 0.0):  # PID
            error = int(self.needTime - self.averageTime)
            if self.needTime >= delta_time:
                speed = 1546
            else:
                speed = int(1549-error*0.08)

            if self.speed < 1546:
                self.speed = 1546
            elif self.speed > 1555:
                self.speed = 1555

            self.pi.set_servo_pulsewidth(self.motorGPIO, self.speed)
        mutex.release()

    def start(self):
        mutex.acquire()
        self.lastEventTime = time.monotonic()
        GPIO.add_event_detect(self.encoderGPIO, GPIO.RISING, callback=self.update)
        mutex.release()
        print('Encoder_Started')

    def stop(self):
        mutex.acquire()
        GPIO.remove_event_detect(self.encoderGPIO)
        self.stopped = True
        mutex.release()
        print('Encoder_Stoped')


# encoder = Encoder()
# encoder.start()
# encoder.set_motor_control(ESC, pi, 300)
# encoder.start_speed_control()
