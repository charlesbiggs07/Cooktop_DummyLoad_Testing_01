import serial
import threading
import csv
import time
from datetime import datetime

PORT = 'COM3'  # Replace with your actual port (e.g., /dev/ttyUSB0 on Linux)
BAUDRATE = 9600
FILENAME = f"Filename_{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.csv"

logging = False
stop_program = False

def listen_for_input(ser):
    global logging, stop_program
    while not stop_program:
        cmd = input("Type 'start', 'stop', or 'quit': ").strip().lower()
        if cmd == 'start':
            ser.write(b'start\n')
            logging = True
            print("Logging started.")
        elif cmd == 'stop':
            ser.write(b'stop\n')
            logging = False
            print("Logging stopped.")
        elif cmd == 'quit':
            ser.write(b'stop\n')
            logging = False
            stop_program = True
            print("Exiting program...")
        else:
            print("Unknown command")

def main():
    global logging, stop_program

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Connected to {PORT} at {BAUDRATE} baud.")
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        return

    input_thread = threading.Thread(target=listen_for_input, args=(ser,))
    input_thread.daemon = True
    input_thread.start()

    with open(FILENAME, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'data'])

        while not stop_program:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if logging and line:
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    writer.writerow([timestamp, line])
                    print(f"[{timestamp}] {line}")

    ser.close()
    print("Serial port closed.")

if __name__ == '__main__':
    main()
