import imufusion
import matplotlib.pyplot as plt
import numpy as np
import sys
import scipy.integrate 
from filterpy.kalman import KalmanFilter
from scipy import signal

kf = KalmanFilter(dim_x=6, dim_z=3)  # Assuming a 6-dimensional state and 3-dimensional measurement
kf.x = np.zeros(6)  # Initial state estimate
kf.F = np.eye(6)  # State transition matrix (assuming identity matrix)
kf.H = np.eye(3, 6)  # Measurement matrix (assuming identity matrix)
kf.P *= 10  # Covariance matrix
kf.R = np.eye(3) * 0.1  # Measurement noise covariance
kf.Q = np.eye(6) * 0.01  # Process noise covariance

# Import sensor data
delta_time = 1 / 84.28
data = np.genfromtxt("move_left.csv", delimiter=",")

timestamp = np.arange(len(data)) * (delta_time)

gyroscope = data[:, 0:3] * 2 * np.pi / (360 * 131.0)
accelerometer = data[:, 3:6] / 16384.0


velocity = np.cumsum(accelerometer, axis=0) * (delta_time)
position = np.cumsum(velocity, axis=0) * (delta_time)

# Plot sensor data
fig, axes = plt.subplots(nrows=3, ncols=2, sharex=True)

axes[0, 0].plot(timestamp, gyroscope[:, 0], "tab:red", label="X")
axes[0, 0].plot(timestamp, gyroscope[:, 1], "tab:green", label="Y")
axes[0, 0].plot(timestamp, gyroscope[:, 2], "tab:blue", label="Z")
axes[0, 0].set_title("Gyroscope")
axes[0, 0].set_ylabel("Degrees/s")
axes[0, 0].grid()
axes[0, 0].legend()

axes[1, 0].plot(timestamp, accelerometer[:, 0], "tab:red", label="X")
axes[1, 0].plot(timestamp, accelerometer[:, 1], "tab:green", label="Y")
axes[1, 0].plot(timestamp, accelerometer[:, 2], "tab:blue", label="Z")
axes[1, 0].set_title("Accelerometer")
axes[1, 0].set_ylabel("g")
axes[1, 0].grid()
axes[1, 0].legend()

# Process sensor data
ahrs = imufusion.Ahrs()
euler = np.empty((len(timestamp), 3))
linacc = np.empty((len(timestamp), 3))

min_vel = float("inf")
max_vel = float("-inf")

for index in range(len(timestamp)):
    kf.predict()
    kf.update(accelerometer[index])
    filtered_state = kf.x

    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], delta_time)  # 100 Hz sample rate
    euler[index] = ahrs.quaternion.to_euler()
    linacc[index] = ahrs.earth_acceleration
    linacc[index, 2] = linacc[index, 2] + 0.983
    linacc[index] = linacc[index] * 9.81

    velocity[:index+1] = np.cumsum(linacc[:index+1], axis=0) * delta_time

    #try high pass filter
    N = 1
    filter_cutoff = 0.1
    Wn = (2*filter_cutoff)/(1/delta_time)
    sos = signal.butter(N, Wn, btype='highpass', output='sos')
    filtered_vel = signal.sosfilt(sos, velocity)

    print(np.linalg.norm(filtered_vel) - np.linalg.norm(velocity))
    
    kf.predict()
    kf.update(velocity[index])
    filtered_state = kf.x

    position[:index+1] = np.cumsum(filtered_vel[:index+1], axis=0) * delta_time

    #try high pass filter
    N = 1
    filter_cutoff = 0.1
    Wn = (2*filter_cutoff)/(1/delta_time)
    sos = signal.butter(N, Wn, btype='highpass', output='sos')
    filtered_position = signal.sosfilt(sos, position)
    print(np.linalg.norm(filtered_position) - np.linalg.norm(position))

    kf.predict()
    kf.update(position[index])
    filtered_state = kf.x


# Plot Euler angles
axes[2, 0].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[2, 0].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[2, 0].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[2, 0].set_title("Euler angles")
axes[2, 0].set_ylabel("Degrees")
axes[2, 0].grid()
axes[2, 0].legend()

# Plot linear acceleration
axes[0, 1].plot(timestamp, linacc[:, 0], "tab:red", label="X")
axes[0, 1].plot(timestamp, linacc[:, 1], "tab:green", label="Y")
axes[0, 1].plot(timestamp, linacc[:, 2], "tab:blue", label="Z")
axes[0, 1].set_title("Linear Acceleration")
axes[0, 1].set_ylabel("g")
axes[0, 1].grid()
axes[0, 1].legend()

# Plot position
axes[1, 1].plot(timestamp, filtered_vel[:, 0], "tab:red", label="X")
axes[1, 1].plot(timestamp, filtered_vel[:, 1], "tab:green", label="Y")
axes[1, 1].plot(timestamp, filtered_vel[:, 2], "tab:blue", label="Z")
axes[1, 1].set_title("velocity")
axes[1, 1].set_xlabel("Seconds")
axes[1, 1].set_ylabel("m/s")
axes[1, 1].grid()
axes[1, 1].legend()

# Plot position
axes[2, 1].plot(timestamp, filtered_position[:, 0], "tab:red", label="X")
axes[2, 1].plot(timestamp, filtered_position[:, 1], "tab:green", label="Y")
axes[2, 1].plot(timestamp, filtered_position[:, 2], "tab:blue", label="Z")
axes[2, 1].set_title("position")
axes[2, 1].set_xlabel("Seconds")
axes[2, 1].set_ylabel("m")
axes[2, 1].grid()
axes[2, 1].legend()

plt.tight_layout()
plt.show(block="no_block" not in sys.argv)  # don't block when script run by CI

