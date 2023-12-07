import imufusion
import matplotlib.pyplot as pyplot
import numpy as np
import sys

# Import sensor data
data = np.genfromtxt("imu_data.csv", delimiter=",")

timestamp = np.arange(len(data)) * (1 / 84.28)
gyroscope = data[:, 0:3] * 2 * np.pi / (360 * 131.0)
accelerometer = data[:, 3:6] / 16384.0

# Plot sensor data
_, axes = pyplot.subplots(nrows=4, sharex=True)

axes[0].plot(timestamp, gyroscope[:, 0], "tab:red", label="X")
axes[0].plot(timestamp, gyroscope[:, 1], "tab:green", label="Y")
axes[0].plot(timestamp, gyroscope[:, 2], "tab:blue", label="Z")
axes[0].set_title("Gyroscope")
axes[0].set_ylabel("Degrees/s")
axes[0].grid()
axes[0].legend()

axes[1].plot(timestamp, accelerometer[:, 0], "tab:red", label="X")
axes[1].plot(timestamp, accelerometer[:, 1], "tab:green", label="Y")
axes[1].plot(timestamp, accelerometer[:, 2], "tab:blue", label="Z")
axes[1].set_title("Accelerometer")
axes[1].set_ylabel("g")
axes[1].grid()
axes[1].legend()

# Process sensor data
ahrs = imufusion.Ahrs()
euler = np.empty((len(timestamp), 3))
linacc = np.empty((len(timestamp), 3))

#print(dir(ahrs))

for index in range(len(timestamp)):
    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], 1 / 84.28)  # 100 Hz sample rate
    euler[index] = ahrs.quaternion.to_euler()
    linacc[index] = ahrs.earth_acceleration

# Plot Euler angles
axes[2].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[2].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[2].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[2].set_title("Euler angles")
axes[2].set_xlabel("Seconds")
axes[2].set_ylabel("Degrees")
axes[2].grid()
axes[2].legend()

print(np.shape(linacc))

# Plot Euler angles
axes[3].plot(timestamp, linacc[:, 0], "tab:red", label="X")
axes[3].plot(timestamp, linacc[:, 1], "tab:green", label="Y")
axes[3].plot(timestamp, linacc[:, 2], "tab:blue", label="Z")
axes[3].set_title("Linear Acceleration angles")
axes[3].set_xlabel("Seconds")
axes[3].set_ylabel("g")
axes[3].grid()
axes[3].legend()

pyplot.show(block="no_block" not in sys.argv)  # don't block when script run by CI
