import serial
import socket
import time
import re
import sys

# Configuration
SERIAL_PORT = "/dev/cu.usbmodem1101"  # Adjust as needed
BAUD_RATE = 921600 # Matches src/main.cpp
UDP_IP = "127.0.0.1"
UDP_PORT = 9870

def hex_to_signed(hex_str):
    """Converts a 16-bit hex string to a signed integer."""
    val = int(hex_str, 16)
    if val > 0x7FFF:
        val -= 0x10000
    return val

def main():
    # Setup UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        print(f"Sending UDP to {UDP_IP}:{UDP_PORT}")
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        sys.exit(1)

    # Regex to parse the line: "X:FF70 Y:FFE0 Z:0050  Angle: 4.9 deg"
    # Handles variable whitespace
    pattern = re.compile(r'X:([0-9A-Fa-f]+)\s+Y:([0-9A-Fa-f]+)\s+Z:([0-9A-Fa-f]+)\s+Angle:\s*([0-9.-]+)')

    try:
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                        
                    match = pattern.search(line)
                    if match:
                        raw_x, raw_y, raw_z, angle_str = match.groups()
                        
                        x = hex_to_signed(raw_x)
                        y = hex_to_signed(raw_y)
                        z = hex_to_signed(raw_z)
                        angle = float(angle_str)
                        
                        # InfluxDB Line Protocol: measurement,tag_set field_set timestamp
                        # Here: "imu x=...,y=...,z=...,angle=..."
                        # We can omit timestamp to let the receiver assign it, or add it if needed.
                        # PlotJuggler with UDP input often expects JSON or just the text, but user asked for Influx.
                        
                        message = f"imu x={x},y={y},z={z},angle={angle}"
                        
                        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
                        # print(f"Sent: {message}") # Optional: debug print
                        
                except ValueError:
                    pass # parse error
                except Exception as e:
                    print(f"Error processing line: {e}")
                    
            else:
                time.sleep(0.001) # slight delay to not hog CPU
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()

