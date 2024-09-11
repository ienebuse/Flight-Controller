import os
import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from ui_blackBoxViewer import *
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from bbDataLoader import *

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QPainter, QPen


class VelLabel(QLabel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.x = 0
        self.y = 0
        # self.angle = initial_angle  # Initial angle in degrees
        # self.length = length  # Length of the line

        # # Create a QTimer to update the angle
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_angle)  # Connect the timer to the update function
        # self.timer.start(1000)  # Update the angle every 1000 milliseconds (1 second)

        # self.setText("Dynamic Line Angle")  # Optional: Add text to the label
        # self.setAlignment(Qt.AlignCenter)  # Center the text

    def update_vel(self, x, y):
        self.x = x
        self.y = y

        self.update()  # Trigger a repaint by calling update()

    def paintEvent(self, event):
        # Call the base class's paintEvent to ensure the label is drawn correctly
        super().paintEvent(event)

        # Now start painting the line
        painter = QPainter(self)

        # Set pen for the line
        pen = QPen(Qt.green, 2, Qt.SolidLine)
        painter.setPen(pen)

        # Calculate the center point of the label
        center_x = self.pos().x() + self.width() // 2
        center_y = self.pos().y() + self.height() // 2

        # Calculate the end point of the line based on the angle and length
        end_x = center_x + self.x
        end_y = center_y + self.y # Minus because y-axis is downwards

        # Draw the line from the center to the calculated end point
        painter.drawLine(QPoint(center_x, center_y), QPoint(int(end_x), int(end_y)))

        # Ensure painter is properly ended
        painter.end()

class Motor():
    def __init__(self, motor: QLabel, scale):
        self.motor:QLabel = motor
        self.defaultPos = motor.pos()
        self.defaultHeight = motor.height()
        self.defaultWidth = motor.width()
        self.scale = scale

        self.currentVal = 0

    def setMotorValue(self,mVal):
        val = int(mVal * self.scale)
        self.currentVal = mVal
        self.motor.setGeometry(self.defaultPos.x(), self.defaultPos.y() - val, self.defaultWidth, self.defaultHeight+val)

    def setScale(self, scale):
        self.scale = scale
        val = self.scale * self.currentVal
        self.motor.setGeometry(self.defaultPos.x(), self.defaultPos.y() - val, self.defaultWidth, self.defaultHeight+val)

class Block(FigureCanvas):
    def __init__(self, parent):
        # self.fig, self.ax = plt.subplots(111,figsize=(parent.width()/100,parent.height()/100), )

        self.fig = plt.figure(figsize=(parent.width()/100,parent.height()/100))
        self.ax = self.fig.add_subplot(111, projection='3d')

        super().__init__(self.fig)
        self.setParent(parent)


        self.original_vertices  = self.create_block()
        
        # Set up the plot
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])

        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        # self.ax.spines['left'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)

        # plt.subplots_adjust(left=0.0, right=1) 

        # Optionally, remove ticks
        self.ax.xaxis.set_ticks([])
        self.ax.yaxis.set_ticks([])
        self.ax.zaxis.set_ticks([])

        self.fig.canvas.draw()
        
        # Define colors for opposite faces
        self.colors = ['blue', 'green', 'red', 'black']

        # Initialize plot
        self.plot_block(self.original_vertices)

    def create_block(self):
        # vertices = np.array([
        #     [-0.5, -0.5, -0.5],
        #     [ 0.5, -0.5, -0.5],
        #     [ 0.5,  0.5, -0.5],
        #     [-0.5,  0.5, -0.5],
        #     [-0.5, -0.5,  0.5],
        #     [ 0.5, -0.5,  0.5],
        #     [ 0.5,  0.5,  0.5],
        #     [-0.5,  0.5,  0.5]
        # ])

        vertices = np.array([
            [-1, -1, -1],
            [ 1, -1, -1],
            [ 1,  1, -1],
            [-1,  1, -1],
            [-1, -1,  1],
            [ 1, -1,  1],
            [ 1,  1,  1],
            [-1,  1,  1]
        ])
        return vertices

    def rotation_matrix(self,roll, pitch, yaw):
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
    
    def rotate_block(self, roll, pitch, yaw):
        R = self.rotation_matrix(roll, pitch, yaw)
        rotated_vertices = np.dot(self.original_vertices, R.T)
        self.plot_block(rotated_vertices)
    
    def create_faces(self,vertices):
        faces = [
            [vertices[j] for j in [0, 1, 5, 4]],
            [vertices[j] for j in [1, 2, 6, 5]],
            [vertices[j] for j in [2, 3, 7, 6]],
            [vertices[j] for j in [3, 0, 4, 7]],
            [vertices[j] for j in [0, 1, 2, 3]],
            [vertices[j] for j in [4, 5, 6, 7]]
        ]
        return faces
    
    def plot_block(self, vertices):
        faces = self.create_faces(vertices)
        c = 0
        # Clear the current polygons
        self.ax.clear()
        for i, face in enumerate(faces):
            poly3d = Poly3DCollection([face], linewidths=1, edgecolors='k')
            # poly3d.set_facecolor(colors[i // 2])
            if(i == 0):
                c = 3
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
            
            poly3d.set_facecolor(self.colors[c])
            if i == 3:
                c = 1
            self.ax.add_collection3d(poly3d)
        self.ax.set_xlabel('X', labelpad=1, color='white')
        self.ax.set_ylabel('Y', labelpad=1, color='white')
        self.ax.set_zlabel('Z', labelpad=1, color='white')
        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])

        # self.ax.xaxis.label.set_position([10.5, -0.1])
        # self.ax.yaxis.label.set_position([0.5, -0.1])
        # self.ax.zaxis.label.set_position([0.5, -0.1])

        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        # self.ax.spines['left'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)

        self.fig.patch.set_alpha(0.1)  # Transparent background for the figure
        # self.ax.patch.set_alpha(0.0)   # Transparent background for the plot
        self.fig.patch.set_facecolor('grey')  # Figure background to transparent
        self.ax.set_facecolor('black')         # Axes background to transparent

        plt.subplots_adjust(top=1, bottom=0, left=0.0, right=1) 

        # Optionally, remove ticks
        self.ax.xaxis.set_ticks([])
        self.ax.yaxis.set_ticks([])
        self.ax.zaxis.set_ticks([])

        plt.tight_layout()
        self.fig.canvas.draw()




class Canvas(FigureCanvas):
    def __init__(self, parent):
        self.fig, self.ax = plt.subplots(figsize=(parent.width()/100,parent.height()/100))
        # self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)
        self.setParent(parent)
        # self.fig.set_size_inches(parent.width()/self.fig.dpi, parent.height()/self.fig.dpi)
        self.data = []
        self.dataNames = []
        self.dataColor = [None, None, None, None]
        self.cursorPos = None

        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        # self.ax.spines['left'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)

        # Optionally, remove ticks
        self.ax.xaxis.set_ticks([])
        self.ax.yaxis.set_ticks([])
        self.fig.patch.set_alpha(0.1)  # Transparent background for the figure
        # self.ax.patch.set_alpha(0.0)   # Transparent background for the plot
        self.fig.patch.set_facecolor('grey')  # Figure background to transparent
        self.ax.set_facecolor('none')         # Axes background to transparent
        plt.subplots_adjust(left=0.0, right=1) 

        self.range = [0,1]      

    def addData(self, data, name):
        self.data.append(data)
        self.dataNames.append(name)
        color = self.plot()
        return len(self.data) - 1, self.dataColor[len(self.data) - 1]

    def removeData(self,index):
        if(index > len(self.data)-1):
            return
        self.data.pop(index)
        self.dataNames.pop(index)
        color = self.plot()
        return self.dataNames, color

    def plot(self):
        self.ax.clear()
        self.fig.canvas.draw()
        for i,data in enumerate(self.data):
            line, = self.ax.plot(data[int(self.range[0]):int(self.range[1])])
            self.dataColor[i] = line.get_color()
        self.ax.draw(self.get_renderer())
        self.repaint()
        # plt.show()
        return self.dataColor

    def setRange(self, range):
        self.range = range
        self.plot()

    def setCursor(self, pos, offset):
        if(len(self.data) == 0):
            return
        if(self.cursorPos != None):
            self.cursorPos.remove()
        self.cursorPos = self.ax.axvline(pos, color='r', linestyle='--', label='Vertical Line')
        self.fig.canvas.draw()
        self.ax.draw(self.get_renderer())
        self.repaint()
        return [round(data[int(offset + pos)],4) for data in self.data]

    def removeCursor(self):
        self.cursorPos.remove()
        self.fig.canvas.draw()
        self.ax.draw(self.get_renderer())
        self.repaint()


class MainWindow(QMainWindow):
    def __init__(self,parent=None):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.dataLoaded = False
        self.mouseDragged = False
        self.mouseDown = False
        self.mouseReleased = False
        self.mouseUp = False
        self.cursorDown = False
        self.start_point = None
        self.end_point = None
        self.windowOffset = 0
        self.winsowSize = 0
        self.tstamp = []
        self.paramIndex = 0

        self.chart1Names = [self.ui.p10N, self.ui.p11N, self.ui.p12N, self.ui.p13N]
        self.chart2Names = [self.ui.p20N, self.ui.p21N, self.ui.p22N, self.ui.p23N]
        self.chart3Names = [self.ui.p30N, self.ui.p31N, self.ui.p32N, self.ui.p33N]
        self.chart4Names = [self.ui.p40N, self.ui.p41N, self.ui.p42N, self.ui.p43N]

        self.chart1Values = [self.ui.p10V, self.ui.p11V, self.ui.p12V, self.ui.p13V]
        self.chart2Values = [self.ui.p20V, self.ui.p21V, self.ui.p22V, self.ui.p23V]
        self.chart3Values = [self.ui.p30V, self.ui.p31V, self.ui.p32V, self.ui.p33V]
        self.chart4Values = [self.ui.p40V, self.ui.p41V, self.ui.p42V, self.ui.p43V]

        self.ui.rstPlotBtn.clicked.connect(self.on_resetPlotBtnClicked)
        self.ui.loadDataBtn.clicked.connect(self.loadBlkBxData)
        self.ui.paramList.currentIndexChanged.connect(self.newParamSelected)
        self.ui.p0.clicked.connect(self.on_p0Clicked)
        self.ui.p1.clicked.connect(self.on_p1Clicked)
        self.ui.p2.clicked.connect(self.on_p2Clicked)
        self.ui.p3.clicked.connect(self.on_p3Clicked)
        self.ui.p4.clicked.connect(self.on_p4Clicked)

        self.ui.mtrScale.currentIndexChanged.connect(self.on_motorScaleChanged)
        self.motor1 = Motor(self.ui.mtr1, float(self.ui.mtrScale.currentText()))
        self.motor2 = Motor(self.ui.mtr2, float(self.ui.mtrScale.currentText()))
        self.motor3 = Motor(self.ui.mtr3, float(self.ui.mtrScale.currentText()))
        self.motor4 = Motor(self.ui.mtr4, float(self.ui.mtrScale.currentText()))

        self.lPos = self.ui.lSlider.value()
        self.ui.lSlider.valueChanged.connect(self.on_lSliderValueChanged)
        # self.ui.lSlider.sliderPressed.connect(self.on_lSliderValueChanged)

        # self.hPos = self.ui.hSlider.value()
        # self.ui.hSlider.sliderMoved.connect(self.on_hSliderValueChanged)
        
        self.ui.lSlider.setEnabled(False)
        # self.ui.hSlider.setEnabled(False)

        self.blockView = Block(self.ui.blockWidget)

        self.chart1 = Canvas(self.ui.pane0)
        self.chart2 = Canvas(self.ui.pane1)
        self.chart3 = Canvas(self.ui.pane2)
        self.chart4 = Canvas(self.ui.pane3)

        self.chart1.mpl_connect('button_press_event', self.on_mousePressed)
        self.chart2.mpl_connect('button_press_event', self.on_mousePressed)
        self.chart3.mpl_connect('button_press_event', self.on_mousePressed)
        self.chart4.mpl_connect('button_press_event', self.on_mousePressed)

        self.chart1.mpl_connect('button_release_event', self.on_mouseReleased)
        self.chart2.mpl_connect('button_release_event', self.on_mouseReleased)
        self.chart3.mpl_connect('button_release_event', self.on_mouseReleased)
        self.chart4.mpl_connect('button_release_event', self.on_mouseReleased)

        self.chart1.mpl_connect('motion_notify_event', self.on_mouseDragged)
        self.chart2.mpl_connect('motion_notify_event', self.on_mouseDragged)
        self.chart3.mpl_connect('motion_notify_event', self.on_mouseDragged)
        self.chart4.mpl_connect('motion_notify_event', self.on_mouseDragged)

        self.vel_centre_x = self.ui.droneView.width()//2 + self.ui.droneView.pos().x()
        self.vel_centre_y = self.ui.droneView.height()//2 + self.ui.droneView.pos().y()

        self.velView = VelLabel(self.ui.droneView)
        self.velView.setFixedSize(self.ui.droneView.width(), self.ui.droneView.height())

        self.show()

    def on_motorScaleChanged(self):
        self.motor1.setScale(float(self.ui.mtrScale.currentText()))
        self.motor2.setScale(float(self.ui.mtrScale.currentText()))
        self.motor3.setScale(float(self.ui.mtrScale.currentText()))
        self.motor4.setScale(float(self.ui.mtrScale.currentText()))

    def on_resetPlotBtnClicked(self):
        self.chart1.setRange([0,len(self.tstamp)])
        self.chart2.setRange([0,len(self.tstamp)])
        self.chart3.setRange([0,len(self.tstamp)])
        self.chart4.setRange([0,len(self.tstamp)])
        self.windowOffset = 0
        self.windowSize = len(self.tstamp)
        self.ui.lSlider.setValue(self.windowOffset)
        
    def on_mousePressed(self, event):
        if(event.button == 1):
            self.lPos = int(event.xdata)
            self.mouseDown = True
            self.start_point = QPoint(event.xdata, event.ydata)
            # self.update()
            print(f"Left button clicked at: {event.xdata}, {event.ydata}")
        elif(event.button == 3):
            self.cursorDown = True
            print(f"Right button clicked at: {event.xdata}, {event.ydata}")
            # self.update()

    def on_mouseReleased(self, event):
        if(event.button == 1):
            if(self.mouseDragged):
                self.mouseDown = False
                self.mouseDragged = False
                self.mouseReleased = True
                self.hPos = int(event.xdata)

                if(self.lPos > self.hPos):
                    temp = self.lPos
                    self.lPos = self.hPos
                    self.hPos = temp

                self.lPos = max(0,self.lPos)
                self.hPos = min(len(self.tstamp),self.hPos)

                self.chart1.setRange([self.lPos + self.windowOffset,self.hPos + self.windowOffset])
                self.chart2.setRange([self.lPos + self.windowOffset,self.hPos + self.windowOffset])
                self.chart3.setRange([self.lPos + self.windowOffset,self.hPos + self.windowOffset])
                self.chart4.setRange([self.lPos + self.windowOffset,self.hPos + self.windowOffset])  
                
                self.windowOffset = self.windowOffset + self.lPos
                self.ui.lSlider.setValue(self.windowOffset)
                self.windowSize = self.hPos - self.lPos
                

                print(f"Left button released at: {event.xdata}, {event.ydata}")
                # self.update()
            else:
                self.mouseDown = False
                self.mouseDragged = False
                # self.update()
        
        elif(event.button == 3):
            self.cursorDown = False
            print(f"Right button released at: {event.xdata}, {event.ydata}")
            # self.update()

    def on_mouseDragged(self, event):
        if(self.mouseDown):
            self.mouseDragged = True
            self.end_point = QPoint(event.xdata, event.ydata)
            print('Left mouse dragged')
            # self.update()
        elif(self.cursorDown and event.xdata != None):
            pos = max(0,event.xdata)
            pos = min(pos, self.windowSize)
            val1 = self.chart1.setCursor(pos, self.windowOffset)
            val2 = self.chart2.setCursor(pos, self.windowOffset)
            val3 = self.chart3.setCursor(pos, self.windowOffset)
            val4 = self.chart4.setCursor(pos, self.windowOffset)

            if(val1 != None):
                for i, v in enumerate(val1):
                    self.chart1Values[i].setText(str(v))

            if(val2 != None):
                for i, v in enumerate(val2):
                    self.chart2Values[i].setText(str(v))

            if(val3 != None):
                for i, v in enumerate(val3):
                    self.chart3Values[i].setText(str(v))

            if(val4 != None):
                for i, v in enumerate(val4):
                    self.chart4Values[i].setText(str(v))
            print('Right cursor dragged')
            self.updateBlock(int(pos + self.windowOffset))
            self.updateMotor(int(pos + self.windowOffset))
            self.updateVelocity(int(pos + self.windowOffset))
            
            # self.update()

    def updateBlock(self, pos):
        roll = params[0].data[pos]
        pitch = params[1].data[pos]
        yaw = params[2].data[pos]

        self.blockView.rotate_block(roll, pitch, yaw)

    def updateMotor(self, pos):
        self.motor1.setMotorValue(params[3].data[pos])
        self.motor2.setMotorValue(params[4].data[pos])
        self.motor3.setMotorValue(params[5].data[pos])
        self.motor4.setMotorValue(params[6].data[pos])

    def updateVelocity(self, pos):
        self.velView.update_vel(params[15].data[pos]/5,params[16].data[pos]/5)
        print(params[15].data[pos],params[16].data[pos])

    def setVel(self):
        # super().paintEvent(event)
        # if self.mouseDown and self.start_point is not None and self.end_point is not None:
        #     painter = QPainter(self)
        #     pen = QPen(Qt.red, 2, Qt.SolidLine)  # Customize pen for line
        #     painter.setPen(pen)
        #     # Draw the line only along the x-axis
        #     painter.drawLine(self.start_point.x(), self.start_point.y(), self.end_point.x(), self.start_point.y())
        #     painter.end()

        # painter = QPainter(self)
        # pen = QPen(Qt.green, 2, Qt.SolidLine)  # Customize pen for line
        # painter.setPen(pen)
        # painter.drawLine(QPoint(self.vel_centre_x, self.vel_centre_y), QPoint(int(self.vel_centre_x + 10), int(self.vel_centre_y + 10)))
        # painter.end()
        pass



    def loadBlkBxData(self):
        self.tstamp = load_data()
        self.ui.lSlider.setEnabled(True)
        # self.ui.hSlider.setEnabled(True)
        # self.ui.lSlider.minimum = self.tstamp[0]
        self.ui.lSlider.setMaximum(len(self.tstamp)-1)
        # self.ui.lSlider.setValue(self.ui.lSlider.minimum())
        self.ui.lSlider.setTickInterval(len(self.tstamp)/100)
        self.lPos = 0
        self.hPos = len(self.tstamp)
        self.dataLoaded = True

        # self.ui.hSlider.minimum = self.tstamp[0]
        # self.ui.hSlider.setMaximum(len(self.tstamp)-1)
        # self.ui.hSlider.setValue((self.ui.lSlider.maximum()))
        # self.ui.hSlider.setTickInterval(len(self.tstamp)/100) 
        # self.hPos = self.ui.hSlider.value()
        
        self.chart1.setRange([self.lPos, self.hPos])
        self.chart2.setRange([self.lPos, self.hPos])
        self.chart3.setRange([self.lPos, self.hPos])
        self.chart4.setRange([self.lPos, self.hPos])

        self.ui.lSlider.setValue(self.windowOffset)
        self.windowSize = len(self.tstamp)

        print('data loaded')
    
    def newParamSelected(self):
        self.paramIndex = self.ui.paramList.currentIndex()
        panel = params[self.paramIndex].panel
        if(panel == 0):
            self.ui.p0.setChecked(True)
        if(panel == 1):
            self.ui.p1.setChecked(True)
        if(panel == 2):
            self.ui.p2.setChecked(True)
        if(panel == 3):
            self.ui.p3.setChecked(True)
        if(panel == 4):
            self.ui.p4.setChecked(True)

    def removeParam(self):
        panel = params[self.paramIndex].panel
        names = None
        if(panel == 1):
            names,color = self.chart1.removeData(params[self.paramIndex].pIndx)
            for n in range(4):
                self.chart1Names[n].setText('')
                self.chart1Names[n].setStyleSheet('')
                self.chart1Values[n].setText('')
            if(names != None):
                for n in range(len(names)):
                    self.chart1Names[n].setText( names[n])
                    self.chart1Names[n].setStyleSheet(f"background-color: {color[n]}; color: black;")
        elif(panel == 2):
            names,color = self.chart2.removeData(params[self.paramIndex].pIndx)
            for n in range(4):
                self.chart2Names[n].setText('')
                self.chart2Names[n].setStyleSheet('')
                self.chart2Values[n].setText('')
            if(names != None):
                for n in range(len(names)):
                    self.chart2Names[n].setText( names[n])
                    self.chart2Names[n].setStyleSheet(f"background-color: {color[n]}; color: black;")
        elif(panel == 3):
            names,color = self.chart3.removeData(params[self.paramIndex].pIndx)
            for n in range(4):
                self.chart3Names[n].setText('')
                self.chart3Names[n].setStyleSheet('')
                self.chart3Values[n].setText('')
            if(names != None):
                for n in range(len(names)):
                    self.chart3Names[n].setText( names[n])
                    self.chart3Names[n].setStyleSheet(f"background-color: {color[n]}; color: black;")
        elif(panel == 4):
            names,color = self.chart4.removeData(params[self.paramIndex].pIndx)
            for n in range(4):
                self.chart4Names[n].setText('')
                self.chart4Names[n].setStyleSheet('')
                self.chart4Values[n].setText('')
            if(names != None):
                for n in range(len(names)):
                    self.chart4Names[n].setText( names[n])
                    self.chart4Names[n].setStyleSheet(f"background-color: {color[n]}; color: black;")
        params[self.paramIndex].panel = 0
        params[self.paramIndex].pIndx = 0

        return names

    def on_p0Clicked(self):  
        self.removeParam()
        # if(names != None):
        #     for n in range(len(names)):
        #         self.chart1Names[n] = names[n]    

    def on_p1Clicked(self):
        self.removeParam()
        # if(names != None):
        #     for n in range(len(names)):
        #         self.chart1Names[n] = names[n]
        name = self.ui.paramList.currentText()
        pIndx, color = self.chart1.addData(params[self.paramIndex].data, name)
        params[self.paramIndex].pIndx = pIndx
        params[self.paramIndex].panel = 1
        self.chart1Names[params[self.paramIndex].pIndx].setText(name)
        self.chart1Names[params[self.paramIndex].pIndx].setStyleSheet(f"background-color: {color}; color: black;")

    def on_p2Clicked(self):
        self.removeParam()
        # if(names != None):
        #     for n in range(len(names)):
        #         self.chart1Names[n] = names[n]
        name = self.ui.paramList.currentText()
        pIndx, color = self.chart2.addData(params[self.paramIndex].data, name)
        params[self.paramIndex].pIndx = pIndx
        params[self.paramIndex].panel = 2
        self.chart2Names[params[self.paramIndex].pIndx].setText(name)
        self.chart2Names[params[self.paramIndex].pIndx].setStyleSheet(f"background-color: {color}; color: black;")

    def on_p3Clicked(self):
        self.removeParam()
        # if(names != None):
        #     for n in range(len(names)):
        #         self.chart1Names[n] = names[n]
        name = self.ui.paramList.currentText()
        pIndx, color = self.chart3.addData(params[self.paramIndex].data, name)
        params[self.paramIndex].pIndx = pIndx
        params[self.paramIndex].panel = 3
        self.chart3Names[params[self.paramIndex].pIndx].setText(name)
        self.chart3Names[params[self.paramIndex].pIndx].setStyleSheet(f"background-color: {color}; color: black;")

    def on_p4Clicked(self):
        self.removeParam()
        # if(names != None):
        #     for n in range(len(names)):
        #         self.chart1Names[n] = names[n]
        name = self.ui.paramList.currentText()
        pIndx, color = self.chart4.addData(params[self.paramIndex].data, name)
        params[self.paramIndex].pIndx = pIndx
        params[self.paramIndex].panel = 4
        self.chart4Names[params[self.paramIndex].pIndx].setText(name)
        self.chart4Names[params[self.paramIndex].pIndx].setStyleSheet(f"background-color: {color}; color: black;")

    def on_lSliderValueChanged(self,value):
        if(self.windowSize + value >= len(self.tstamp)):
            self.ui.lSlider.setValue(self.windowOffset)
        else:
            if(self.mouseReleased):
                self.mouseReleased = False
                return
            self.windowOffset = value
            
            self.chart1.setRange([self.windowOffset, self.windowOffset + self.windowSize])
            self.chart2.setRange([self.windowOffset, self.windowOffset + self.windowSize])
            self.chart3.setRange([self.windowOffset, self.windowOffset + self.windowSize])
            self.chart4.setRange([self.windowOffset, self.windowOffset + self.windowSize])

        # if(value >= self.hPos - 1):
        #     self.ui.lSlider.setValue(value-5)
        # self.ui.lPos.setText(str(self.tstamp[self.ui.lSlider.value()]))
        # self.ui.lPos.adjustSize()
        # self.lPos = self.ui.lSlider.value()
        # self.chart1.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart2.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart3.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart4.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])

    def on_hSliderValueChanged(self,value):
        pass
        # if(value <= self.lPos + 1):
        #     self.ui.hSlider.setValue(value+5)
        # self.ui.hPos.setText(str(self.tstamp[self.ui.hSlider.value()]))
        # self.ui.hPos.adjustSize()
        # self.hPos = self.ui.hSlider.value()
        # self.chart1.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart2.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart3.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])
        # self.chart4.setRange([self.ui.lSlider.value(), self.ui.hSlider.value()])


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())