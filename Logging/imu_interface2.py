import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import threading
import time
import serial
import signal
from matplotlib.ticker import MultipleLocator

import struct


showBlock = False
# showBlock = True
rxData = []
completeData = []
dataRdy = False
pcktIndex = 0

# Parameters
window_size = 1000  # Number of data points to display in the window
update_interval = 20  # Update interval in milliseconds

# Initialize data for 4 subplots
x_data = list(range(window_size))
Roll = [0] * window_size
Pitch = [0] * window_size
Yaw = [0] * window_size
Heading = [0] * window_size

rMin = pMin = yMin = hMin = 10000000000
rMax = pMax = yMax = hMax = -10000000000


# ser = serial.Serial('COM9', 9600)
# ser = serial.Serial('COM9', 115200)
ser = serial.Serial('COM16', 256000)

should_stop = False

stop_event = threading.Event()

# Signal handler for SIGINT
def signal_handler(sig, frame):
    global should_stop
    print('Ctrl+C detected. Stopping...')
    stop_event.set()
    should_stop = True

# Function to create the vertices of a unit block centered at the origin
def create_block():
    vertices = np.array([
        [-0.5, -0.5, -0.5],
        [ 0.5, -0.5, -0.5],
        [ 0.5,  0.5, -0.5],
        [-0.5,  0.5, -0.5],
        [-0.5, -0.5,  0.5],
        [ 0.5, -0.5,  0.5],
        [ 0.5,  0.5,  0.5],
        [-0.5,  0.5,  0.5]
    ])
    return vertices

# Function to create the faces of the block for plotting
def create_faces(vertices):
    faces = [
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [2, 3, 7, 6]],
        [vertices[j] for j in [3, 0, 4, 7]],
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]]
    ]
    return faces

# Function to create the rotation matrix for roll, pitch, and yaw
def rotation_matrix(roll, pitch, yaw):
    # Convert angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    # Rotation matrix for roll
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation matrix for pitch
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation matrix for yaw
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))
    
    return R

# Function to apply the rotation to the block
def rotate_block(vertices, roll, pitch, yaw):
    R = rotation_matrix(roll, pitch, yaw)
    rotated_vertices = np.dot(vertices, R.T)
    return rotated_vertices

# Function to plot the block
def plot_block(ax, vertices, colors):
    faces = create_faces(vertices)
    c = 0
    # Clear the current polygons
    ax.clear()
    for i, face in enumerate(faces):
        poly3d = Poly3DCollection([face], linewidths=1, edgecolors='k')
        # poly3d.set_facecolor(colors[i // 2])
        if(i == 0):
            c = 0
        elif(i == 1):
            c = 1
        elif(i == 2):
            c = 0
        elif(i == 3):
            c = 1
        elif(i == 4):
            c = 2
        elif(i == 5):
            c = 2
        
        poly3d.set_facecolor(colors[c])
        if i == 3:
            c = 1
        ax.add_collection3d(poly3d)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    # ax.view_init(elev=20, azim=30)  # Set initial view angle

# Global variables to store roll, pitch, and yaw
roll, pitch, yaw, heading, dT, ch, bat, altitude, GPS, v = 0, 0, 0, 0, 0, [0,0,0,0,0,0,0,0,0,0,0,0,0], 0, 0, 0, 0

# ser = None

State_Idle = 0
State_Receiving = 1
State_Escaped1 = 2
State_Escaped2 = 3

timeout = False

BYTE_START_FLAG1 = 0x8E
BYTE_START_FLAG2 = 0x81
BYTE_END_FLAG1 = 0xA5
BYTE_END_FLAG2 = 0xAE

def TimeOut(signum, frame):
    global timeout
    print("No response from PIB")
    timeout = True


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

LOG_DATA_SIZE = 34
R = P = Y = heading = dT = M1 = M2 = M3 = M4 = Rp = Ri = Rd  = Pp = Pi = Pd = Yp = Yi = Yd = chR = chP =chT = chY = SL1 = SL2 = SR2 = SR1 = ofVx = ofVy = ofH = bat = alt = lat = lon = hMSL = 0
def processData(rxData):
    global dataRdy, R, P, Y, heading, dT, M1, M2, M3, M4, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL
    global completeData, pcktIndex
    data = rxData[:-2]
    if(len(data) != LOG_DATA_SIZE):
        return
    rxCRC = rxData[-2]
    calcCRC = calculate_checksum(data)
    if(rxCRC == calcCRC):
        indx = data[0]
        if(indx != pcktIndex):
            pcktIndex = 0
            completeData = []
            return
        completeData  = completeData + data[1:] 
        if(pcktIndex == 3):      
            _, R, P, Y, heading, dT, M1, M2, M3, M4, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL = struct.unpack('<I5f17f4h3f5f',bytearray(completeData))
            dataRdy = True
            completeData = []
        pcktIndex  = (pcktIndex + 1 ) % 4
        pass

# RX_DATA_SIZE = 134
RX_DATA_SIZE = 36

def uart_listerner():
    global ser, rxData, timeout
    state = State_Idle
    while(not timeout): 
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
                            processData(rxData)
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


# Thread function to update roll, pitch, and yaw values
def update_angles():
    global dataRdy, R, P, Y, heading, dT, M1, M2, M3, M4, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL

    while not stop_event.is_set() and not should_stop:
        if(dataRdy):
            time.sleep(0.1)

# Main function to simulate the motion of the block
def simulate_block_motion(interval):
    # Create the block
    if(showBlock):
        original_vertices  = create_block()
        
        # Set up the plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        
        # Define colors for opposite faces
        colors = ['blue', 'green', 'red', 'black']

        # Initialize plot
        plot_block(ax, original_vertices, colors)
    else:
        # Create figure and axes for 4 subplots
        fig, axs = plt.subplots(4, 1, figsize=(10, 8))
        lines = []

        # Set up each subplot
        for ax in axs:
            ax.clear()
            line, = ax.plot(x_data, [0] * window_size)
            ax.set_xlim(0, window_size - 1)
            # ax.autoscale(enable=True, axis='y', tight=False)
            # ax.set_ylim(-1, 1)
            lines.append(line)

    # Define an update function for animation
    def updateBlock(i):
        # nonlocal vertices
        global dataRdy, R, P, Y, heading, dT, M1, M2, M3, M4, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL
        vertices = rotate_block(original_vertices, R, P, Y)
        plot_block(ax, vertices, colors)
        try:
            print(f"Frame {i}: R={R:.2f},\t P={P:.2f},\t Y={Y:.2f},\t Heading={heading:.1f},\t dT={dT:.6f}\t M1:{M1:.1f}   M2:{M2:.1f}   M3:{M3:.1f}   M4:{M4:.1f}\t bat:{bat:.1f}  H:{alt:.3f} lat:{lat:.3f} | lon:{lon:.3f} | hMSL:{hMSL:.3f}     vx:{ofVx:.3f} | vy:{ofVy:.3f} | h:{ofH:.3f}          ", end='\r')
            # print(f"Frame {i}: R={R:.2f},\t P={P:.2f},\t Y={Y:.2f},\t Heading={heading:.1f},\t dT={dT:.6f}\t M1:{M1:.1f}   M2:{M2:.1f}   M3:{M3:.1f}   M4:{M4:.1f}  R:[{Rp:.3f} | {Ri:.3f} | {Rd:.3f}]   P:[{Pp:.3f} | {Pi:.3f} | {Pd:.3f}]    Y:[{Yp:.3f} | {Yi:.3f} | {Yd:.3f}]\t bat:{bat:.1f}  H:{alt:.3f} lat:{lat:.3f} | lon:{lon:.3f} | hMSL:{hMSL:.3f}     vx:{ofVx:.3f} | vy:{ofVy:.3f} | h:{ofH:.3f}          ", end='\r')
            # print(f"Frame {i}: R={R:.2f},\t P={P:.2f},\t Y={Y:.2f},\t Heading={heading:.1f},\t dT={dT:.6f}\t M1:{M1:.1f}   M2:{M2:.1f}   M3:{M3:.1f}   M4:{M4:.1f}   \
            #       R:[{Rp:.3f} | {Ri:.3f} | {Rd:.3f}]   P:[{Pp:.3f} | {Pi:.3f} | {Pd:.3f}]    Y:[{Yp:.3f} | {Yi:.3f} | {Yd:.3f}]\t bat:{bat:.1f}  H:{alt:.3f}   \
            #         lat:{lat:.3f} | lon:{lon:.3f} | hMSL:{hMSL:.3f}     vx:{ofVx:.3f} | vy:{ofVy:.3f} | h:{ofH:.3f}          ", end='\r')
        except:
            pass

        return ax,


    # Update function
    def updateLine(i):
        global Roll, Pitch, Yaw, Heading ,dataRdy, R, P, Y, heading, dT, M1, M2, M3, M4, Rp, Ri, Rd , Pp, Pi, Pd, Yp, Yi, Yd, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL
        # Get new data point (replace this with your actual data source)        
        tPID = (M1 + M2 + M3 + M4)/4
        # Update data

        # Roll.append(ofVx)
        # Pitch.append(ofVy)
        # Yaw.append(ofH)
        # Heading.append(alt)

        Roll.append(M1)
        Pitch.append(M2)
        Yaw.append(M4)
        Heading.append(tPID)


        Roll = Roll[1:]#.pop(0)
        Pitch = Pitch[1:]#.pop(0)
        Yaw = Yaw[1:]#.pop(0)
        Heading = Heading[1:]#.pop(0)

        rMin = min(Roll)
        pMin = min(Pitch)
        yMin = min(Yaw)
        hMin = min(Heading)
        rMax = max(Roll)
        pMax = max(Pitch)
        yMax = max(Yaw)
        hMax = max(Heading)

        # axs[0].clear()
        # axs[1].clear()
        # axs[2].clear()
        # axs[3].clear()

        axs[0].set_ylim(rMin-0.5*abs(rMin), rMax+0.5*abs(rMax))
        axs[1].set_ylim(pMin-0.5*abs(pMin), pMax+0.5*abs(pMax))
        axs[2].set_ylim(yMin-0.5*abs(yMin), yMax+0.5*abs(yMax))
        axs[3].set_ylim(hMin-0.1*abs(hMin), hMax+0.1*abs(hMax))


        # try:
        #     # lines[0].autoscale(enable=True, axis='y', tight=False)
        #     # lines[1].autoscale(enable=True, axis='y', tight=False)
        #     # lines[2].autoscale(enable=True, axis='y', tight=False)
        #     # lines[3].autoscale(enable=True, axis='y', tight=False)

        #     axs[0].yaxis.set_major_locator(MultipleLocator((rMax - rMin) / 5))
        #     axs[1].yaxis.set_major_locator(MultipleLocator((pMax - pMin) / 5))
        #     axs[2].yaxis.set_major_locator(MultipleLocator((yMax - yMin) / 5))
        #     axs[3].yaxis.set_major_locator(MultipleLocator((hMax - hMin) / 5))
        # except:
        #     pass
        
        # # Update plot data
        lines[0].set_ydata(Roll)
        lines[1].set_ydata(Pitch)
        lines[2].set_ydata(Yaw)
        lines[3].set_ydata(Heading)

        # axs[0].plot(Roll)
        # axs[0].plot(Pitch)
        # axs[0].plot(Yaw)
        # axs[0].plot(Heading)

        # axs[0].set_ylabel('Vx')
        # axs[1].set_ylabel('Vy')
        # axs[2].set_ylabel('H')
        # axs[3].set_ylabel('Alt')
        try:
            # lines[0].set_ylabel(f'{round(Roll[-1],2)}')
            # lines[1].set_ylabel(f'{round(Pitch[-1],2)}')
            # lines[2].set_ylabe(f'{round(Yaw[-1],2)}')
            # lines[3].set_ylabel(f'{round(Heading[-1],2)}')
            axs[0].set_title(f'{round(Roll[-1],2)}')
            axs[1].set_title(f'{round(Pitch[-1],2)}')
            axs[2].set_title(f'{round(Yaw[-1],2)}')
            axs[3].set_title(f'{round(Heading[-1],2)}')
        except:
            pass

        

        try:
            
            print(f"Frame {i}: R={R:.2f},\t P={P:.2f},\t Y={Y:.2f},\t Hd={heading:.1f},\t dT={dT:.6f}\t M1:{M1:.1f}   M2:{M2:.1f}   M3:{M3:.1f}   M4:{M4:.1f}   tPID:{(float)(tPID):.1f}\tvx:{ofVx:.3f} | vy:{ofVy:.3f} | h:{ofH:.1f}    bat:{bat:.1f}  H:{alt:.3f} lat:{lat:.3f} | lon:{lon:.3f} | hMSL:{hMSL:.3f}          ", end='\r')
            # print(f"Frame {i}: R={R:.2f},\t P={P:.2f},\t Y={Y:.2f},\t Heading={heading:.1f},\t dT={dT:.6f}\t M1:{M1:.1f}   M2:{M2:.1f}   M3:{M3:.1f}   M4:{M4:.1f}  R:[{Rp:.3f} | {Ri:.3f} | {Rd:.3f}]   P:[{Pp:.3f} | {Pi:.3f} | {Pd:.3f}]    Y:[{Yp:.3f} | {Yi:.3f} | {Yd:.3f}]\t bat:{bat:.1f}  H:{alt:.3f} lat:{lat:.3f} | lon:{lon:.3f} | hMSL:{hMSL:.3f}     vx:{ofVx:.3f} | vy:{ofVy:.3f} | h:{ofH:.3f}          ", end='\r')
        except:
            pass

        return lines
    
    # Create animation
    # anim = FuncAnimation(fig, updateBlock, frames=range(1000), interval=interval, blit=True)
    anim = FuncAnimation(fig, updateBlock if showBlock else updateLine, frames=range(1000), interval=interval, blit=True)
    
    plt.show()

# Start the thread to update angles
# angle_thread = threading.Thread(target=update_angles)
# angle_thread.daemon = True
# angle_thread.start()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)    
    # Example usage to create motion with 100ms interval
    listener = threading.Thread(target=uart_listerner)
    listener.daemon = True
    listener.start()
    simulate_block_motion(1)
