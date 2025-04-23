import iio
import numpy as np
import time
import matplotlib.pyplot as plt

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


# Initialize the Kalman filter for pitch
kf_pitch = KalmanFilter(0.02, 0.4)  # Process and measurement noise for pitch

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

    return x_accel, y_accel, z_accel

# Function to calculate pitch from accelerometer data
def calculate_pitch(x_accel, y_accel, z_accel):
    pitch = np.arctan2(y_accel, np.sqrt(x_accel**2 + z_accel**2)) * 180 / np.pi
    return pitch

# Lists to store raw and filtered pitch data
raw_pitch_data = []
filtered_pitch_data = []

# Main loop to read IMU data, process with Kalman filter, and display results
try:
    while True:
        x_accel, y_accel, z_accel = read_imu()

        # Calculate the pitch from accelerometer data
        accel_pitch = calculate_pitch(x_accel, y_accel, z_accel)

        # Apply the Kalman filter to the accelerometer data for more stable results
        pitch = kf_pitch.update(accel_pitch)

        # Store raw and filtered data
        raw_pitch_data.append(accel_pitch)
        filtered_pitch_data.append(pitch)

        # Plot the results (every 10 samples)
        if len(raw_pitch_data) % 10 == 0:
            plt.clf()  # Clear the figure
            plt.plot(raw_pitch_data, 'o', label="Raw Pitch Angle")
            plt.plot(filtered_pitch_data, '-', label="Filtered Pitch Angle")
            plt.xlabel("Time (samples)")
            plt.ylabel("Pitch Angle (°)")
            plt.legend(loc="best")
            plt.title("Kalman Filtered vs Raw Pitch Angle")

            # Save the plot as a high-resolution image
            plt.savefig("pitch_angle_comparison.png", dpi=300)  # Save at 300 DPI
            print("Plot saved as 'pitch_angle_comparison.png'")

        time.sleep(0.02)  # Small delay for continuous reading
except KeyboardInterrupt:
    print("\nStopped by user")
