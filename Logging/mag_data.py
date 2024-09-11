import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import threading
import time
import serial
import signal
import csv

stop_event = threading.Event()
should_stop = False
csv_file_path = 'mag_data_top.csv'


ser = serial.Serial('COM14', 256000)
rows = []

# Signal handler for SIGINT
def signal_handler(sig, frame):
    global should_stop
    print('Ctrl+C detected. Stopping...')
    stop_event.set()
    should_stop = True


# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)    
count = 0
while not should_stop:
    data = ser.read_until()
    try:
        row = data.decode().strip().split(',')
        rows.append(row)
        print(row)
        count = count + 1
    except:
        continue


# Open the CSV file for writing
with open(csv_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Collect and write sensor data
    for r in rows:  # Collecting 100 samples as an example
        # Read sensor data
        writer.writerow([float(i) for i in r])

print(f"Data saved to {csv_file_path}")