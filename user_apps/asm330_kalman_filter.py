import iio
import numpy as np
import time

# Kalman filter class
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        self.posteri_estimate = 0.0  # Initial estimate of angle
        self.posteri_error_estimate = 1.0  # Initial error estimate

    def update(self, measurement):
        # Prediction step
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        # Update step
        kalman_gain = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + kalman_gain * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - kalman_gain) * priori_error_estimate

        return self.posteri_estimate


# Initialize the Kalman filter for pitch and roll
kf_pitch = KalmanFilter(0.1, 0.1)  # Process and measurement noise for pitch
kf_roll = KalmanFilter(0.1, 0.1)   # Process and measurement noise for roll

# Initialize IIO context and device
ctx = iio.Context()
device = ctx.find_device("asm330_iio")
if device is None:
    raise RuntimeError("IMU device 'asm330_iio' not found!")

# Function to read accelerometer and gyroscope data
def read_imu():
    x_accel = int(device.find_channel("accel_x").attrs["raw"].value)
    y_accel = int(device.find_channel("accel_y").attrs["raw"].value)
    z_accel = int(device.find_channel("accel_z").attrs["raw"].value)

    x_gyro = int(device.find_channel("anglvel_x").attrs["raw"].value)
    y_gyro = int(device.find_channel("anglvel_y").attrs["raw"].value)
    z_gyro = int(device.find_channel("anglvel_z").attrs["raw"].value)

    return x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro

# Function to calculate pitch and roll from accelerometer data
def calculate_angles(x_accel, y_accel, z_accel):
    pitch = np.arctan2(y_accel, np.sqrt(x_accel**2 + z_accel**2)) * 180 / np.pi
    roll = np.arctan2(-x_accel, np.sqrt(y_accel**2 + z_accel**2)) * 180 / np.pi
    return pitch, roll

# Main loop to read IMU data, process with Kalman filter, and display results
try:
    while True:
        x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro = read_imu()

        # Calculate the pitch and roll from accelerometer data
        accel_pitch, accel_roll = calculate_angles(x_accel, y_accel, z_accel)

        # Apply the Kalman filter to the accelerometer data for more stable results
        pitch = kf_pitch.update(accel_pitch)
        roll = kf_roll.update(accel_roll)

        print(f"Pitch: {pitch:.2f}° | Roll: {roll:.2f}°")

        time.sleep(0.1)  # Small delay for continuous reading
except KeyboardInterrupt:
    print("\nStopped by user")
