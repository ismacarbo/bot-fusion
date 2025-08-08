import serial
import time

# Replace with the correct port if different
PORT = "/dev/ttyACM0"
BAUDRATE = 9600

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset

    print(f"[INFO] Connected to {PORT} at {BAUDRATE} baud.")
    print("[INFO] Reading data (Press Ctrl+C to exit)...\n")

    while True:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"Arduino says: {line}")

except serial.SerialException as e:
    print(f"[ERROR] Could not open serial port: {e}")
except KeyboardInterrupt:
    print("\n[INFO] Manual exit.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("[INFO] Serial port closed.")
