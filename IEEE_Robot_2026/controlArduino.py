import serial
import time

# --- Setup serial port ---
arduino_port = "/dev/ttyACM1"
baud_rate = 115200
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

time.sleep(2)  # wait for Arduino to reset

# --- Command functions ---
def move_motor(steps: int):
    if not 0 <= steps <= 255:
        raise ValueError("Steps must be 0-255 for now")
    ser.write(bytes([0x01, steps]))
    ack = ser.read()
    if ack == b'\xAA':
        print(f"Motor moved {steps} steps")
    else:
        print("Error moving motor")

def set_relay(on: bool):
    ser.write(bytes([0x02, 0x01 if on else 0x00]))
    ack = ser.read()
    if ack == b'\xAA':
        print(f"Relay {'ON' if on else 'OFF'}")
    else:
        print("Error setting relay")

# --- Example Usage ---
move_motor(100)  # move 100 steps
set_relay(True)  # turn relay ON
time.sleep(2)
set_relay(False) # turn relay OFF
