import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import threading
import time
import serial
import signal
from matplotlib.ticker import MultipleLocator
import csv

import struct


showBlock = False
# showBlock = True
rxData = []
completeData = []
dataRdy = False
pcktIndex = 0

BLACKBOX_REQ = 0xE9

State_Idle = 0
State_Receiving = 1
State_Escaped1 = 2
State_Escaped2 = 3


BYTE_START_FLAG1 = 0x85
BYTE_START_FLAG2 = 0x8B
BYTE_END_FLAG1 = 0xA5
BYTE_END_FLAG2 = 0xAE

TimeOut_Period = 10


# ser = serial.Serial('COM9', 9600)
# ser = serial.Serial('COM9', 115200)
ser = serial.Serial('COM16', 256000)

should_stop = False

stop_event = threading.Event()




def crc8_dvb_s2(crc, a):
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
        crc &= 0xFF  # Ensure crc is always 8 bits
    return crc

def calculate_checksum(data):
    crc = 0
    for byte in data:
        crc = crc8_dvb_s2(crc, byte)
    return crc


LOG_DATA_SIZE = 130
RX_DATA_SIZE = LOG_DATA_SIZE + 1
BLACKBOX_DATA = []

def write_file():
    global BLACKBOX_DATA, should_stop
    
    print('writing log file...')
    with open('blackbox_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the data rows
        for row in BLACKBOX_DATA[:-1]:
            writer.writerow(row)
    should_stop = True
    print('writing log file complete')

# Signal handler for SIGINT
def signal_handler(sig, frame):
    global should_stop
    print('Ctrl+C detected. Stopping...')
    stop_event.set()
    should_stop = True
    write_file()


# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)    


R = P = Y = heading = dT = M1 = M2 = M3 = M4 = pX = pY = pR = pP = pYw = pT = Rp = Ri = Rd  = Pp = Pi = Pd = Yp = Yi = Yd = chR = chP =chT = chY = SL1 = SL2 = SR2 = SR1 = ofVx = ofVy = ofH = ofPx = ofPy = ofWx = ofWy = bat = alt = lat = lon = hMSL = 0
t2 = 0
first = True
def processData(rxData):
    global t2, should_stop,BLACKBOX_DATA, dataRdy, R, P, Y, heading, dT, M1, M2, M3, M4, pX, pY, pR, pP, pYw, pT, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, ofPx, ofPy, ofWx, ofWy, bat, alt, lat, lon, hMSL
    
    if(len(rxData) != LOG_DATA_SIZE):
        return
    rxCRC = rxData[-1]
    calcCRC = calculate_checksum(rxData[:-1])
    if(rxCRC == calcCRC):  
        t,id, R, P, Y, M1, M2, M3, M4, pX, pY, pR, pP, pYw, pT, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, ofPx, ofPy, ofWx, ofWy, bat, alt, lat, lon, hMSL = struct.unpack('<IB3f4f6f4f4h7f5f',bytearray(rxData[:-1]))
        
        # tPID = (M1 + M2 + M3 + M4)/4
        # rPID = (M1 - M2 + M3 - M4)/4
        # pPID = (M1 + M2 - M3 - M4)/4
        # yPID = (M1 - M2 - M3 + M4)/4
        
        BLACKBOX_DATA.append([t,id, R, P, Y, M1, M2, M3, M4, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, ofPx, ofPy, ofWx, ofWy, bat, alt, lat, lon, hMSL, pT, pR, pP, pYw, pX, pY])
        print(f'log size: {len(BLACKBOX_DATA)}               ', end='\r')
        if(id == 0xD8):
            write_file()

# RX_DATA_SIZE = 134


def uart_listerner():
    global ser, rxData, should_stop
    state = State_Idle
    while(not should_stop): 
        if(ser != None): 
            resp = ser.read(1)
            if(len(resp) > 0):
                data = resp[0]
                if(state == State_Idle):
                    if(data == BYTE_START_FLAG1):
                        rxData = []
                        state = State_Escaped1
                elif(state == State_Escaped1):
                    if(data == BYTE_START_FLAG2):
                        state = State_Receiving
                    else:
                        state == State_Idle
                elif(state == State_Receiving):
                    if(data == BYTE_END_FLAG1):
                        rxData.append(data)
                        state = State_Escaped2
                    else:
                        rxData.append(data)
                elif(state == State_Escaped2):
                    if(data == BYTE_END_FLAG2):
                        if(len(rxData) == RX_DATA_SIZE):
                            processData(rxData[:-1])
                            state = State_Idle
                        elif(len(rxData) < RX_DATA_SIZE):
                            rxData.append(data)
                            state = State_Receiving
                        else:
                            state = State_Idle
                else:
                    state = State_Idle
                    rxData = []
    ser.close()


if __name__ == '__main__':
    # signal.signal(signal.SIGINT, signal_handler)    
    # Example usage to create motion with 100ms interval
    # signal.signal(signal.SIGALRM, TimeOut)

    listener = threading.Thread(target=uart_listerner)
    listener.daemon = True
    listener.start()

    # signal.alarm(TimeOut_Period)

    # data = struct.pack('<B',BLACKBOX_REQ)
    # ser.write(data)
    while(not should_stop):
        time.sleep(1)

    pass