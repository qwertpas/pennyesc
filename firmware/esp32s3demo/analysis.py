import matplotlib.pyplot as plt
import pandas as pd

# Read the CSV file
df = pd.read_csv('sixstepdata.csv')

# Extract IMU x and y columns
imu_x = df['/imu/x']
imu_y = df['/imu/y']

# Create scatter plot
plt.figure(figsize=(10, 8))
plt.scatter(imu_x, imu_y, alpha=0.6, s=20)
plt.xlabel('IMU X')
plt.ylabel('IMU Y')
plt.title('IMU X vs Y Scatter Plot')
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Create time series plot
time = df['__time']
plt.figure(figsize=(12, 6))
plt.scatter(time, imu_x, label='IMU X', alpha=0.7)
plt.scatter(time, imu_y, label='IMU Y', alpha=0.7)
plt.xlabel('Time')
plt.ylabel('IMU Value')
plt.title('IMU X and Y Time Series')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

