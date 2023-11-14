import serial
import time
from pynput import keyboard


def sendToArduino(sendStr):
    ser.write(sendStr.encode("utf-8"))  # change for Python3


def recvFromArduino():
    global startMarker, endMarker

    ck = ""
    x = "z"  # any value that is not an end- or startMarker
    byteCount = -1  # to allow for the fact that the last increment will be one too many

    # wait for the start character
    while ord(x) != startMarker:
        x = ser.read()

    # save data until the end marker is found
    while ord(x) != endMarker:
        if ord(x) != startMarker:
            ck = ck + x.decode("utf-8")  # change for Python3
            byteCount += 1
        x = ser.read()

    return ck

def waitForArduino():
    # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded

    global startMarker, endMarker

    msg = ""
    while msg.find("Arduino is ready") == -1:
        while ser.inWaiting() == 0:
            pass

        msg = recvFromArduino()

        print(msg)  # python3 requires parenthesis
        print()

def on_press(key):
    try:
        # print(f"Key pressed: {key}")
        if key == keyboard.Key.up:
            # print("Sending forward")
            sendToArduino("<FORWARD,0,0>\n")
        elif key == keyboard.Key.down:
            # print("Sending backward")
            sendToArduino("<BACKWARD,0,0>\n")
        elif key == keyboard.Key.left:
            # print("Sending left")
            sendToArduino("<LEFT,0,0>\n")
        elif key == keyboard.Key.right:
            # print("Sending right")
            sendToArduino("<RIGHT,0,0>\n")
        elif key.char == "a":
            sendToArduino("<LSHIFT,0,0>\n")
        elif key.char == "d":
            sendToArduino("<RSHIFT,0,0>\n")
    except AttributeError:
        pass  # Ignore special keys


def on_release(key):
    sendToArduino("<STOP,0,0>\n")
    if key == keyboard.Key.esc:
        return False  # Stop listener


def readFromArduino():
    while True:
        if ser.inWaiting() > 0:
            dataRecvd = recvFromArduino()
            print("Reply Received " + dataRecvd)

serPort = "/dev/ttyACM0"
baudRate = 14400
ser = serial.Serial(serPort, baudRate)
print("Serial port " + serPort + " opened  Baudrate " + str(baudRate))


startMarker = 60
endMarker = 62
# waitForArduino()

print("listening keyboard")
# Keyboard listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Read from Arduino in a loop
readFromArduino()

ser.close
