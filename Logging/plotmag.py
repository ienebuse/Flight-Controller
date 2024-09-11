import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

# Define the calibration data
class CalibData:
    def __init__(self):
        # self.scale = np.array([[1.005367, -0.000931, -0.000440],
        #                        [-0.000931, 0.989576, -0.015483],
        #                        [-0.000440, -0.015483, 1.018926]])
        # self.offset = np.array([-0.989077, 157.014939, 126.731613])

        self.scale = np.array([[9.963706, 0.000776, -0.003530],
                               [0.000776, 9.983607, 0.007309],
                               [-0.003530, 0.007309, 9.958600]])
        self.offset = np.array([-0.000231, 0.000062, -0.000586])

# Function to read magnetometer data from a file
def read_magnetometer_data(file_path):
    x, y, z = [], [], []
    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            if len(row) == 3:  # Ensure the row has exactly 3 elements
                try:
                    x.append(float(row[0]))
                    y.append(float(row[1]))
                    z.append(float(row[2]))
                except ValueError:
                    print(f"Skipping invalid row: {row}")
    return np.array(x), np.array(y), np.array(z)

# Function to apply calibration to magnetometer data
def apply_calibration(x, y, z, calib_data):
    raw_data = np.vstack((x, y, z)).T
    calibrated_data = np.dot(raw_data - calib_data.offset, calib_data.scale.T)
    return calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2]

# Function to plot calibrated vs uncalibrated data
def plot_data(x, y, z, x_cal, y_cal, z_cal):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot uncalibrated data
    ax.scatter(x, y, z, c='r', marker='o', label='Uncalibrated')
    
    # Plot calibrated data
    ax.scatter(x_cal, y_cal, z_cal, c='b', marker='^', label='Calibrated')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Magnetometer Data: Uncalibrated vs Calibrated')
    ax.legend()

    plt.tight_layout()
    plt.show()

# Main function
def main():
    # Replace with the path to your file
    file_path = 'acc_data.txt'

    # Read the data
    x, y, z = read_magnetometer_data(file_path)

    # Apply calibration
    calib_data = CalibData()
    x_cal, y_cal, z_cal = apply_calibration(x, y, z, calib_data)

    # Plot the data
    plot_data(x, y, z, x_cal, y_cal, z_cal)

if __name__ == "__main__":
    main()
