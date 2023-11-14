import serial
import time
from pynput import keyboard


def sendToArduino(sendStr):
    ser.write(sendStr.encode("utf-8"))  # change for Python3


# ======================================


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


# ============================


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


# ======================================


def runTest(td):
    numLoops = len(td)
    waitingForReply = False

    n = 0
    while n < numLoops:
        teststr = td[n]

        if waitingForReply == False:
            sendToArduino(teststr)
            print("Sent from PC -- LOOP NUM " + str(n) + " TEST STR " + teststr)
            waitingForReply = True

        if waitingForReply == True:
            while ser.inWaiting() == 0:
                pass

            dataRecvd = recvFromArduino()
            print("Reply Received  " + dataRecvd)
            n += 1
            waitingForReply = False

            print("===========")

        time.sleep(5)


def on_press(key):
    try:
        print(f"Key pressed: {key}")
        if key.char == keyboard.Key.up:
            sendToArduino("<FORWARD,0,0>\n")
        elif key.char == keyboard.Key.down:
            sendToArduino("<BACKWARD,0,0>\n")
        elif key.char == keyboard.Key.left:
            sendToArduino("<LEFT,0,0>\n")
        elif key.char == keyboard.Key.right:
            sendToArduino("<RIGHT,0,0>\n")
    except AttributeError:
        pass  # Ignore special keys


def on_release(key):
    if key == keyboard.Key.esc:
        return False  # Stop listener


def readFromArduino():
    while True:
        if ser.inWaiting() > 0:
            dataRecvd = recvFromArduino()
            print("Reply Received " + dataRecvd)

serPort = "/dev/ttyACM0"
baudRate = 9600
ser = serial.Serial(serPort, baudRate)
print("Serial port " + serPort + " opened  Baudrate " + str(baudRate))



startMarker = 60
endMarker = 62
waitForArduino()

# Keyboard listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Read from Arduino in a loop
readFromArduino()

# waitForArduino()


# testData = []
# testData.append("<LED1,200,0.2>")
# testData.append("<LED1,800,0.7>")
# testData.append("<LED2,800,0.5>")
# testData.append("<LED2,200,0.2>")
# testData.append("<LED1,200,0.7>")

# runTest(testData)


ser.close
