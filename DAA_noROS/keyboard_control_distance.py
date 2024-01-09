import serial
import time
import threading
from pynput import keyboard

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.flush()


def on_press(key):
    try:
        if key.char == "w":
            ser.write(b"FORWARD\n")
        elif key.char == "s":
            ser.write(b"BACK\n")
        elif key.char == "a":
            ser.write(b"LEFT\n")
        elif key.char == "d":
            ser.write(b"RIGHT\n")
        elif key.char == "q":
            ser.write(b"STOP\n")
    except AttributeError:
        pass


def on_release(key):
    if key == keyboard.Key.esc:
        return False


def read_distance():
    while True:
        ser.write(b"DISTANCE\n")
        time.sleep(0.5)
        if ser.in_waiting > 0:
            print(ser.readline())
            distance = ser.readline().decode("utf-8").rstrip()
            print("Distance: ", distance, "cm")

distance_thread = threading.Thread(target=read_distance, daemon=True)

# Collect events until released
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    distance_thread.start()
    listener.join()




try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    ser.close()
    pass
