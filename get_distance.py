import serial
import time
import threading
from pynput import keyboard

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.flush()

def read_distance():
    while True:
        ser.write(b"DISTANCE\n")
        time.sleep(0.5)
        if ser.in_waiting > 0:
            print(ser.readline())
            distance = ser.readline().decode("utf-8").rstrip()
            print("Distance: ", distance, "cm")

read_distance()