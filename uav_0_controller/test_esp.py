#!/usr/bin/env python3
import serial
import time

# ESP32-C3 reboots when serial connection is opened
print("Opening serial port (this will reboot the ESP)...")
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

print("Waiting 3 seconds for ESP to boot...")
time.sleep(3)  # Increased from 2 to 3 seconds

# Clear any boot messages
ser.reset_input_buffer()

print("Sending RC command...")
cmd = "RC 1500 1500 1000 1500 1000 1500 1500 1500\n"
ser.write(cmd.encode())
time.sleep(0.1)

# Read responses for 2 seconds
start = time.time()
while time.time() - start < 2:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"ESP: {line}")
    time.sleep(0.01)

print("\nSending continuous commands at 50Hz...")
for i in range(20):
    ser.write(cmd.encode())
    time.sleep(0.02)
    
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"ESP: {line}")

print("Done! Check if LED is GREEN")
ser.close()