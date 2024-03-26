from pyrplidar import PyRPlidar
import time
import numpy as np
lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
lidar.set_motor_pwm(500)
max_size = 300
scan_generator = lidar.start_scan_express(0)

scan_data_angles = np.array([], dtype=float)
scan_data_distance = np.array([], dtype=float)
front_data = []
travel_distance = 0

def rplidar_loop():
    np.set_printoptions(suppress=True)
    scan_data = []
    temp = 0
    global scan_data_angles
    global scan_data_distance
    global front_data
    global travel_distance
    while True:
        for scan in scan_generator():
            if scan.quality != 0:
                scan_data.append([scan.angle, scan.distance])

                if(temp<scan.distance):
                    print("New max value: ",scan.distance)
                    temp = scan.distance

                if len(scan_data) > max_size:

                    scan_data_np = np.array(scan_data)

                    angle_values = scan_data_np[:, 0]
                    filtered_indices = np.where(((angle_values >= 345) & (angle_values <= 360)) | ((angle_values >= 0) & (angle_values <= 15)))
                    front_data = scan_data_np[filtered_indices]

                    scan_data_angles, scan_data_distance = np.hsplit(scan_data_np, 2)


                    scan_data = []

#rplidar_loop()
#rplidar_thread = threading.Thread(target=rplidar_loop,daemon=True)
#rplidar_thread.start()

#{'name': 'Standard', 'max_distance': 3072, 'us_per_sample': 130048, 'ans_type': 'NORMAL'}
#{'name': 'Express', 'max_distance': 3072, 'us_per_sample': 65024, 'ans_type': 'CAPSULED'}
#{'name': 'Boost', 'max_distance': 3072, 'us_per_sample': 32512, 'ans_type': 'ULTRA_CAPSULED'}
#{'name': 'Sensitivity', 'max_distance': 3072, 'us_per_sample': 32512, 'ans_type': 'ULTRA_CAPSULED'}
#{'name': 'Stability', 'max_distance': 3072, 'us_per_sample': 51456, 'ans_type': 'ULTRA_CAPSULED'}