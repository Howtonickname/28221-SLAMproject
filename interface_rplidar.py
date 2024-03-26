from pyrplidar import PyRPlidar
import time
import numpy as np
import matplotlib.pyplot as plt
lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
lidar.set_motor_pwm(500)
max_size = 300
#scan_generator = lidar.start_scan_express(3)
scan_generator = lidar.force_scan()

scan_data_angles = np.array([], dtype=float)
scan_data_distance = np.array([], dtype=float)
front_data = []


def get_scan():
    np.set_printoptions(suppress=True)
    scan_data = []
    global scan_data_angles
    global scan_data_distance
    global front_data

    for count, scan in enumerate(scan_generator()):
        if scan.quality != 0:
            scan_data.append([scan.angle, scan.distance])
            if count > max_size:
                scan_data_np = np.array(scan_data)
                angle_values = scan_data_np[:, 0]
                filtered_indices = np.where(((angle_values >= 345) & (angle_values <= 360)) | ((angle_values >= 0) & (angle_values <= 15)))
                front_data = scan_data_np[filtered_indices]
                scan_data_angles, scan_data_distance = np.hsplit(scan_data_np, 2)
                print(scan_data_distance)
                break


def get_scan_single(angle=0):
    for scan in scan_generator():
        if scan.quality != 0 and angle <= scan.angle < angle + 1:
            return scan.angle, scan.distance


for i in range(60):
    get_scan()
    plt.pause(0.5)









#print(get_scan_single(5))

#rplidar_loop()
#rplidar_thread = threading.Thread(target=rplidar_loop,daemon=True)
#rplidar_thread.start()

#{'name': 'Standard', 'max_distance': 3072, 'us_per_sample': 130048, 'ans_type': 'NORMAL'}
#{'name': 'Express', 'max_distance': 3072, 'us_per_sample': 65024, 'ans_type': 'CAPSULED'}
#{'name': 'Boost', 'max_distance': 3072, 'us_per_sample': 32512, 'ans_type': 'ULTRA_CAPSULED'}
#{'name': 'Sensitivity', 'max_distance': 3072, 'us_per_sample': 32512, 'ans_type': 'ULTRA_CAPSULED'}
#{'name': 'Stability', 'max_distance': 3072, 'us_per_sample': 51456, 'ans_type': 'ULTRA_CAPSULED'}