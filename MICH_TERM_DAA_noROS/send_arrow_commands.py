import serial
from pynput import keyboard

# Set up the serial connection (The port will be different depending on your setup)
ser = serial.Serial("/dev/ttyACM0", 9600)  # Replace '/dev/ttyACM0' with your port


def on_press(key):
    try:
        if key == keyboard.Key.up:
            ser.write(b"FORWARD\n")
            print("Moving forward")
        elif key == keyboard.Key.down:
            ser.write(b"BACKWARD\n")
            print("Moving backward")
        elif key == keyboard.Key.left:
            ser.write(b"LEFT\n")
            print("Turning left")
        elif key == keyboard.Key.right:
            ser.write(b"RIGHT\n")
            print("Turning right")
        elif key == keyboard.Key.esc:
            return False  # Stop listener
    except AttributeError:
        pass


def on_release(key):
    ser.write(b"STOP\n")  # Stop the car on any key release
    print("Stopping")


# Collect events until released
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# Clean up: close serial connection
ser.close()
