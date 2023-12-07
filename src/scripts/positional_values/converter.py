import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
import scipy.integrate 

# Read CSV file
datafile_acc = pd.read_csv("imu_data.csv")  # Update "your_file.csv" with the path to your CSV file
acceleration_columns = datafile_acc.iloc[:, 3:6]  # Extract columns 3 to 6 (0-indexed)

# Convert units and format data
data_acc = acceleration_columns * 0.01  # Convert from gal to m/s^2
time_step = 0.005  # Time step of ground motion data

# Calculate duration vector
N = len(data_acc)
time = np.linspace(0.0, N * time_step, N)

# Calculate velocity and displacement
velocity = scipy.integrate.cumtrapz(data_acc, x=time, axis=0)  # Acceleration to velocity
displacement = scipy.integrate.cumtrapz(velocity, x=time[:-1], axis=0)  # Velocity to displacement

# Plot displacement
plt.plot(time[:-2], displacement)
plt.grid(True)
plt.xlim(0, 30)
plt.ylim(-1, 1)
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.title("Displacement vs. Time")
plt.show()
