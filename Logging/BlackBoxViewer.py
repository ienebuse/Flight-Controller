import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
from matplotlib.widgets import Slider, Button



# Function to read magnetometer data from a file
t=[]
id=[]
R=[]
P=[]
Y=[]
M1=[]
M2=[]
M3=[]
M4=[]
chR=[]
chP=[]
chT=[]
chY=[]
SL1=[]
SL2=[]
SR2=[]
SR1=[]
ofVx=[]
ofVy=[]
ofH=[]
bat=[]
alt=[]
lat=[]
lon=[]
hMSL = []
def load_data(file_path='blackbox_data.csv'):
    x, y, z = [], [], []
    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            try:
                t.append(float(row[0]))
                id.append(float(row[1]))
                R.append(float(row[2]))
                P.append(float(row[3]))
                Y.append(float(row[4]))
                M1.append(float(row[5]))
                M2.append(float(row[6]))
                M3.append(float(row[7]))
                M4.append(float(row[8]))
                chR.append(float(row[9]))
                chP.append(float(row[10]))
                chT.append(float(row[11]))
                chY.append(float(row[12]))
                SL1.append(float(row[13]))
                SL2.append(float(row[14]))
                SR2.append(float(row[15]))
                SR1.append(float(row[16]))
                ofVx.append(float(row[17]))
                ofVy.append(float(row[18]))
                ofH.append(float(row[19]))
                bat.append(float(row[20]))
                alt.append(float(row[21]))
                lat.append(float(row[22]))
                lon.append(float(row[23]))
                hMSL.append(float(row[24]))

            except ValueError:
                print(f"Skipping invalid row: {row}")
    return np.array(x), np.array(y), np.array(z)

load_data()

fig, axs = plt.subplots(4, 1, figsize=(10, 8))

axs[0].plot(ofVx)
axs[0].plot(ofVy)
axs[0].set_ylabel(f'Velocity')
axs[0].legend(['x','y'])


axs[1].plot(ofH)
axs[1].set_ylabel(f'Altitude')

axs[2].plot(M1)
axs[2].plot(M2)
axs[2].plot(M3)
axs[2].plot(M4)
axs[2].set_ylabel(f'Motor')
axs[2].legend(['M1','M2','M3','M4'])

axs[3].plot(R)
axs[3].plot(P)
axs[3].plot(Y)
axs[3].set_ylabel(f'Attitude')
axs[3].legend(['r','p','y'])

plt.show()