# uzyte biblioteki
import time
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import threading
import pandas as pd

# moje pliki
import interface_rplidar as rp_i
import interface_mpu6050 as imu
import interface_l298n as motors

def export_data(x, y):
    current_datetime = datetime.now()
    date = current_datetime.strftime('%Y-%m-%d_%H-%M-%S')
    desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
    file_name = f'dane_{date}.csv'
    file_path = os.path.join(desktop, file_name)
    data = pd.DataFrame({'x': x, 'y': y})
    data.to_csv(file_path, index=False)

# def driving_loop():
#     global device_x
#     global device_y
#     isDriving, finished = True, False
#     while not finished:
#         starting_d = rp_i.get_distance()
#         while isDriving:
#             diff = starting_d - rp_i.get_distance()
#             starting_d = rp_i.get_distance()
#             angle_rad = np.radians(imu.new_rotation_angle % 360)
#             device_y = diff * np.sin(angle_rad)
#             for angle, distance in rp_i.front_data:
#                 if distance < 300:
#                     motors.stop()
#                     isDriving = False
#             motors.move_forward()
#         motors.turn_right(0.5)
#         isDriving = True

x = []
y = []
device_x = 0
device_y = 0

rplidar_thread = threading.Thread(target=rp_i.get_scan, daemon=True)
imu_thread = threading.Thread(target=imu.imu_loop, daemon=True)
#driving_thread = threading.Thread(target=driving_loop, daemon=True)
rplidar_thread.start()
imu_thread.start()

plt.figure(figsize=(8, 8))
plt.title('Calibrating, please wait')
plt.xlabel('X-axis (mm)')
plt.ylabel('Y-axis (mm)')
plt.grid(True)
plt.ion()  # Turn on interactive mode
plt.pause(16)
#driving_thread.start()
plt.title('LIDAR data')
for i in range(10):
    angles_rad = np.radians((rp_i.scan_data_angles + imu.new_rotation_angle) % 360)
    more_x = rp_i.scan_data_distance * np.cos(angles_rad)
    more_y = rp_i.scan_data_distance * np.sin(angles_rad)
    plt.scatter(more_x, more_y, s=10, c='black', alpha=0.5, marker='o', edgecolors='black', linewidths=1.5)
    print("loop")
    x = np.append(x, more_x)
    y = np.append(y, more_y)
    plt.pause(1)
print("export time")
export_data(x, y)
plt.ioff()  # Turn off interactive mode
plt.show()
imu_thread.join()
rplidar_thread.join()
#driving_thread.join()
rp_i.lidar.stop()
rp_i.lidar.set_motor_pwm(0)
rp_i.lidar.disconnect()


