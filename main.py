#uzyte biblioteki
from pyrplidar import PyRPlidar
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import threading
import RPi.GPIO as GPIO
#moje pliki

import interface_rplidar as rp_i
import interface_mpu6050 as imu
import interface_l298n as motors
'''
robot ma zaznaczyć wykryty dystans przed jazdą
jak jedzie na przód ile ten dystans skrócił trzeba zaliczyć jako przebyty dystans
następnie przed obrotem jakaś flaga żeby nie rejestrowało zmian
i jak jest gotowy do jazdy to znowu przerabia to samo
należy dosłownie rozszerzyć kod od jeżdżenia i stale rejestrować gdzie robot jest teraz z pomocą kąta z żyroskopu
oraz dodać do punktów bias w postaci odsunięcia się od 0, 0
'''
def driving_loop():
    #To jest kod dla prostego programu w którym robot jedzie do przodu i skreca w prawo
    #Nie jest w żaden sposób sprzężony z mapowaniem w lidarze
    isDriving, flag2 = True, True
    while flag2:
        while isDriving:
            for angle, distance in rp_i.front_data:
                print("Distance:", distance, "Angle:", angle)
                if distance < 300:
                    motors.stop()
                    isDriving = False
            motors.move_forward()
        motors.turn_right(0.5)
        isDriving = True


x=[]
y=[]
device_x=0
device_y=0
more_x=[]
more_y=[]

rplidar_thread = threading.Thread(target=rp_i.rplidar_loop, daemon=True)
imu_thread = threading.Thread(target=imu.imu_loop, daemon=True)
driving_thread = threading.Thread(target=driving_loop, daemon=True)
imu_thread.start()



plt.figure(figsize=(8, 8))
plt.title('Calibrating, please wait')
plt.xlabel('X-axis (mm)')
plt.ylabel('Y-axis (mm)')
plt.grid(True)
plt.ion()  # Turn on interactive mode
plt.pause(15)
rplidar_thread.start()
driving_thread.start()
plt.title('LIDAR data')
for i in range(60):
    angles_rad = np.radians((rp_i.scan_data_angles-imu.new_rotation_angle) % 360)
    more_x=rp_i.scan_data_distance * np.cos(angles_rad)
    more_y=rp_i.scan_data_distance * np.sin(angles_rad)
    plt.scatter(more_x, more_y, s=10, c='black', alpha=0.5, marker='o', edgecolors='black', linewidths=1.5)
    x.extend(more_x)
    y.extend(more_y)
    plt.pause(1)

plt.ioff()  # Turn off interactive mode
plt.show()

imu_thread.join()
rplidar_thread.join()
driving_thread.join()

rp_i.lidar.stop()
rp_i.lidar.set_motor_pwm(0)
rp_i.lidar.disconnect()

# def driving_loop():
#     #To jest kod dla prostego programu w którym robot jedzie do przodu i skreca w prawo
#     #Nie jest w żaden sposób sprzężony z mapowaniem w lidarze
#     flag, flag2 = True, True
#     while flag2:
#         while flag:
#             for angle, distance in rp_i.front_data:
#                 print("Distance:", distance, "Angle:", angle)
#                 if distance < 300:
#                     GPIO.output(forwR, GPIO.LOW)
#                     GPIO.output(forwL, GPIO.LOW)
#                     flag = False
#             GPIO.output(forwR, GPIO.HIGH)
#             GPIO.output(forwL, GPIO.HIGH)
#         turn_right(0.5)
#         flag = True