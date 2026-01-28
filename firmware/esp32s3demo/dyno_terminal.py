#!/usr/bin/env python3
"""
Dual Motor Dyno Terminal Interface
Connects to vertiqdyno WiFi and sends UDP commands.
Controls both Vertiq and PennyESC motors with combined logging.

Usage:
    python3 dyno_terminal.py

Vertiq Commands:
    T<angle>  - Set target angle (e.g., T10, T-5.5)
    V<volts>  - Set voltage (e.g., V0.3)
    VOLT<n>   - Set voltage control mode with nV (e.g., VOLT2)
    VEL<n>    - Set velocity control mode with n rad/s (e.g., VEL20)
    H/HOME    - Home to 0 rad at 5 rad/s
    v<V1>t<T1>v<V2>t<T2>... - Chained commands (e.g., v4t5v6t15)

PennyESC Commands:
    P:D<duty> - Set duty cycle (-799 to 799), e.g., P:D100
    P:T<rad>  - Set target position in radians, e.g., P:T3.14

Common Commands:
    STOP/OFF  - Stop both motors
    ZERO/Z    - Zero both motor angles
    STATUS/S  - Get status of both motors
    LOG       - Download combined log data (prompts: Y [filename], N, or C to clear)
    quit/exit - Exit terminal
"""

import socket
import struct
import base64
import csv
import sys
import os
import numpy as np
from datetime import datetime

ESP32_IP = "192.168.4.1"  # Default ESP32 SoftAP IP
UDP_PORT = 9870
TIMEOUT = 2.0  # seconds

save_dir = './dyno_logs/'

def create_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)
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

def receive_all(sock, timeout=10.0):
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
    Each sample is 28 bytes:
      v_angle(float), v_vel(float), v_vbat(float), v_set_volts(float),
      p_position(float), p_velocity(float), time_us(uint32)
    """
    try:
        raw_data = base64.b64decode(base64_data)
    except Exception as e:
        print(f"Base64 decode error: {e}")
        return None
    
    # Each sample is 28 bytes: 6 floats (4 bytes each) + 1 uint32 (4 bytes)
    sample_size = 28
    num_samples = len(raw_data) // sample_size
    
    if len(raw_data) % sample_size != 0:
        print(f"Warning: Data size {len(raw_data)} not divisible by {sample_size}")
    
    samples = []
    for i in range(num_samples):
        offset = i * sample_size
        # Little-endian: 6 floats + 1 unsigned int
        v_angle, v_vel, v_vbat, v_set_volts, p_position, p_velocity, time_us = struct.unpack(
            '<ffffffI', raw_data[offset:offset+sample_size]
        )
        samples.append({
            'time_us': time_us,
            'time_ms': time_us / 1000.0,
            'v_angle': v_angle,
            'v_vel': v_vel,
            'v_vbat': v_vbat,
            'v_set_volts': v_set_volts,
            'p_position': p_position,
            'p_velocity': p_velocity
        })
    
    return samples

def save_csv(samples, filename):
    """Save samples to CSV file."""
    fieldnames = ['time_us', 'time_ms', 'v_angle', 'v_vel', 'v_vbat', 'v_set_volts', 'p_position', 'p_velocity']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)
    print(f"Saved {len(samples)} samples to {filename}")

def handle_log_command(sock):
    """Handle the LOG command flow with Y/N/C confirmation."""
    response = send_command(sock, "LOG")
    if response is None:
        print("No response from ESP32")
        return
    
    print(response, end='')
    
    if "NO LOG DATA" in response:
        return
    
    if "Download?" not in response:
        return
    
    raw_input_str = input().strip()
    parts = raw_input_str.split(maxsplit=1)
    confirm = parts[0].upper()
    custom_filename = parts[1] if len(parts) > 1 else None
    
    if confirm in ('C', 'CLEAR'):
        sock.sendto(b'C', (ESP32_IP, UDP_PORT))
        try:
            response = sock.recvfrom(1024)[0].decode()
            print(response, end='')
        except socket.timeout:
            pass
        return
    
    if confirm not in ('Y', 'YES'):
        sock.sendto(b'N', (ESP32_IP, UDP_PORT))
        try:
            response = sock.recvfrom(1024)[0].decode()
            print(response, end='')
        except socket.timeout:
            pass
        return
    
    sock.sendto(b'Y', (ESP32_IP, UDP_PORT))
    
    try:
        ack = sock.recvfrom(1024)[0].decode()
        print(ack, end='')
    except socket.timeout:
        print("No acknowledgment received")
        return
    
    print("Receiving data...")
    base64_data = receive_all(sock, timeout=15.0)
    
    if not base64_data:
        print("No data received")
        return
    
    print(f"Received {len(base64_data)} characters of base64 data")
    
    samples = decode_log_data(base64_data)
    if samples is None:
        print("Failed to decode data")
        return
    
    print(f"Decoded {len(samples)} samples")
    
    if samples:
        duration_ms = samples[-1]['time_ms']
        if len(samples) > 1:
            dts = [samples[i]['time_ms'] - samples[i-1]['time_ms'] for i in range(1, len(samples))]
            median_period = np.median(dts)
            print(f"Duration: {duration_ms:.1f}ms, Median period: {median_period:.3f}ms ({1000/median_period:.0f}Hz)" if median_period > 0 else "")
    
    # Ensure save directory exists
    os.makedirs(save_dir, exist_ok=True)
    
    if custom_filename:
        if not custom_filename.endswith('.csv'):
            custom_filename += '.csv'
        filename = custom_filename
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"dyno_log_{timestamp}.csv"
        
    save_csv(samples, save_dir + filename)

def main():
    print("=" * 60)
    print("Dual Motor Dyno Terminal")
    print(f"Connecting to {ESP32_IP}:{UDP_PORT}")
    print("=" * 60)
    print("Vertiq:   T<angle>, V<volts>, VOLT<n>, VEL<n>, H, v4t5v6t15")
    print("PennyESC: P:D<duty>, P:T<rad>")
    print("Common:   STOP, ZERO, STATUS, LOG")
    print("Type 'quit' or 'exit' to exit")
    print("=" * 60)
    
    sock = create_socket()
    
    print("\nTesting connection...")
    response = send_command(sock, "STATUS")
    if response:
        print(f"Connected!\n{response}")
    else:
        print("Warning: No response from ESP32. Make sure you're connected to vertiqdyno WiFi.")
    
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
        
        if cmd.upper() == 'LOG':
            handle_log_command(sock)
            continue
        
        response = send_command(sock, cmd)
        if response:
            print(response, end='')
        else:
            print("No response (timeout)")
    
    sock.close()

if __name__ == "__main__":
    main()
