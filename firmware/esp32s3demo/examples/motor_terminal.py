#!/usr/bin/env python3
"""
Motor Control Terminal Interface
Connects to dnajumper WiFi and sends UDP commands.
Handles LOG command with base64 decoding and CSV export.

Usage:
    python3 motor_terminal.py

Commands:
    T<angle>  - Set target angle (e.g., T10, T-5.5)
    V<volts>  - Set voltage (e.g., V0.3)
    VOLT<n>   - Set voltage control mode with nV (e.g., VOLT2)
    VEL<n>    - Set velocity control mode with n rad/s (e.g., VEL20)
    H/HOME    - Home to 0 rad at 5 rad/s, then restore previous mode
    PRBS [amp] [bit_ms] [dur_s] - PRBS excitation for sysid (default: 5V, 20ms, 10s)
    STOP/OFF  - Stop motor
    ZERO/Z    - Zero angle
    STATUS/S  - Get status
    LOG       - Download log data (prompts: Y [filename], N, or C to clear)
    quit/exit - Exit terminal

Chained Commands:
    v<V1>t<T1>v<V2>t<T2>... - Chain voltage/target commands without braking
    Example: v4t5v6t15
        - Set voltage to 4V, go to 5 rad
        - On reaching 5 rad, immediately set 6V and continue to 15 rad
        - No braking between segments (smooth transition)
"""

import socket
import struct
import base64
import csv
import sys
import numpy as np
from datetime import datetime

ESP32_IP = "192.168.4.1"  # Default ESP32 SoftAP IP
UDP_PORT = 9870
TIMEOUT = 2.0  # seconds

save_dir = './motor_logs/'

def create_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)
    # Bind to any available port so we can receive replies
    sock.bind(('', 0))
    return sock

def send_command(sock, cmd):
    """Send a command and return the response."""
    sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
    try:
        data, addr = sock.recvfrom(4096)
        return data.decode('utf-8', errors='replace')
    except socket.timeout:
        return None

def receive_all(sock, timeout=5.0):
    """Receive multiple packets until END marker or timeout."""
    sock.settimeout(timeout)
    chunks = []
    
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            text = data.decode('utf-8', errors='replace')
            
            if text.strip() == "END":
                break
            
            chunks.append(text)
            
        except socket.timeout:
            print("Timeout waiting for data")
            break
    
    return ''.join(chunks)

def decode_log_data(base64_data):
    """
    Decode base64 log data into list of samples.
    Each sample is: angle(float), vel(float), vbat(float), set_volts(float), time_us(uint32)
    """
    try:
        raw_data = base64.b64decode(base64_data)
    except Exception as e:
        print(f"Base64 decode error: {e}")
        return None
    
    # Each sample is 20 bytes: 4 floats (4 bytes each) + 1 uint32 (4 bytes)
    sample_size = 20
    num_samples = len(raw_data) // sample_size
    
    if len(raw_data) % sample_size != 0:
        print(f"Warning: Data size {len(raw_data)} not divisible by {sample_size}")
    
    samples = []
    for i in range(num_samples):
        offset = i * sample_size
        # Little-endian: 4 floats + 1 unsigned int
        angle, vel, vbat, set_volts, time_us = struct.unpack('<ffffI', raw_data[offset:offset+sample_size])
        samples.append({
            'time_us': time_us,
            'time_ms': time_us / 1000.0,
            'angle': angle,
            'vel': vel,
            'vbat': vbat,
            'set_volts': set_volts
        })
    
    return samples

def save_csv(samples, filename):
    """Save samples to CSV file."""
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time_us', 'time_ms', 'angle', 'vel', 'vbat', 'set_volts'])
        writer.writeheader()
        writer.writerows(samples)
    print(f"Saved {len(samples)} samples to {filename}")

def handle_log_command(sock):
    """Handle the LOG command flow with Y/N/C confirmation."""
    # Send LOG command
    response = send_command(sock, "LOG")
    if response is None:
        print("No response from ESP32")
        return
    
    print(response, end='')
    
    if "NO LOG DATA" in response:
        return
    
    if "Download?" not in response:
        return
    
    # Prompt for confirmation (Y [filename], N, or C to clear)
    raw_input = input().strip()
    parts = raw_input.split(maxsplit=1)
    confirm = parts[0].upper()
    custom_filename = parts[1] if len(parts) > 1 else None
    
    # Handle C/CLEAR - clear log without downloading
    if confirm in ('C', 'CLEAR'):
        sock.sendto(b'C', (ESP32_IP, UDP_PORT))
        try:
            response = sock.recvfrom(1024)[0].decode()
            print(response, end='')
        except socket.timeout:
            pass
        return
    
    # Handle N or anything other than Y
    if confirm not in ('Y', 'YES'):
        sock.sendto(b'N', (ESP32_IP, UDP_PORT))
        try:
            response = sock.recvfrom(1024)[0].decode()
            print(response, end='')
        except socket.timeout:
            pass
        return
    
    # Send Y confirmation to download (and clear)
    sock.sendto(b'Y', (ESP32_IP, UDP_PORT))
    
    # Wait for "SENDING..." acknowledgment
    try:
        ack = sock.recvfrom(1024)[0].decode()
        print(ack, end='')
    except socket.timeout:
        print("No acknowledgment received")
        return
    
    # Receive all base64 data chunks
    print("Receiving data...")
    base64_data = receive_all(sock, timeout=10.0)
    
    if not base64_data:
        print("No data received")
        return
    
    print(f"Received {len(base64_data)} characters of base64 data")
    
    # Decode
    samples = decode_log_data(base64_data)
    if samples is None:
        print("Failed to decode data")
        return
    
    print(f"Decoded {len(samples)} samples")
    
    if samples:
        duration_ms = samples[-1]['time_ms']
        # Compute sampling frequency using the median of sample intervals (dt)
        if len(samples) > 1:
            dts = [samples[i]['time_ms'] - samples[i-1]['time_ms'] for i in range(1, len(samples))]
            median_period = np.median(dts)
        else:
            median_period = 0
        print(f"Duration: {duration_ms:.1f}ms, Median period: {median_period:.3f}ms ({1000/median_period:.0f}Hz)" if median_period > 0 else "")
    
    # Save to CSV
    if custom_filename:
        if not custom_filename.endswith('.csv'):
            custom_filename += '.csv'
        filename = custom_filename
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_log_{timestamp}.csv"
        
    save_csv(samples, save_dir+filename)

def main():
    print("=" * 50)
    print("Motor Control Terminal")
    print(f"Connecting to {ESP32_IP}:{UDP_PORT}")
    print("Commands: T<angle>, V<volts>, VOLT<n>, VEL<n>, H, PRBS, STOP, ZERO, STATUS, LOG")
    print("Chains:   v<V1>t<T1>v<V2>t<T2>... (e.g., v4t5v6t15)")
    print("Type 'quit' or 'exit' to exit")
    print("=" * 50)
    
    sock = create_socket()
    
    # Test connection with STATUS
    print("\nTesting connection...")
    response = send_command(sock, "STATUS")
    if response:
        print(f"Connected! {response}")
    else:
        print("Warning: No response from ESP32. Make sure you're connected to dnajumper WiFi.")
    
    # Main command loop
    while True:
        try:
            cmd = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break
        
        if not cmd:
            continue
        
        if cmd.lower() in ('quit', 'exit', 'q'):
            print("Exiting...")
            break
        
        # Special handling for LOG command
        if cmd.upper() == 'LOG':
            handle_log_command(sock)
            continue
        
        # Regular command
        response = send_command(sock, cmd)
        if response:
            print(response, end='')
        else:
            print("No response (timeout)")
    
    sock.close()

if __name__ == "__main__":
    main()

