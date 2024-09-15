import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
from matplotlib.widgets import Slider, Button



# Function to read magnetometer data from a file
tstamp=[]
id=[]

NUM_PARAMS = 29

class bbParam():    
    def __init__(self):
        self.data = 0
        self.panel = 0
        self.pIndx = 0
    
    def reset(self):
        self.data = []


params  = [bbParam() for _ in range(NUM_PARAMS)]


def load_data(file_path='blackbox_data.csv'):
    global params,t, id #, R, P, Y, M1, M2, M3, M4, chR, chP,chT, chY, SL1, SL2, SR2, SR1, ofVx, ofVy, ofH, bat, alt, lat, lon, hMSL, tPID, rPID, pPID, yPID, pX, pY,
    tstamp=[]
    id=[]
    for i in range(NUM_PARAMS):
        params[i].reset()

    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            try:
                tstamp.append(float(row[0]))
                id.append(float(row[1]))
                
                for i in range(NUM_PARAMS):
                    params[i].data.append(float(row[i+2]))

            except ValueError:
                print(f"Skipping invalid row: {row}")
    return tstamp