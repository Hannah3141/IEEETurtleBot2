import serial
import time

# --- Setup serial port ---
arduino_port = "/dev/ttyACM1"
baud_rate = 115200
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

time.sleep(2)  # wait for Arduino to reset

# --- Command functions ---
def move_motor(steps: int):
    if not -128 <= steps <= 127:
        raise ValueError("Steps must be between -128 and 127.")
    byte_value = steps & 0xFF
    ser.write(bytes([0x01, byte_value))
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
move_motor(100)  # move 100 steps CW 
move_motor(-20)	 # move 20 steps CCW	
set_relay(True)  # turn relay ON
time.sleep(2)
set_relay(False) # turn relay OFF
