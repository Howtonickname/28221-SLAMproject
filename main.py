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
                #print("Distance:", distance, "Angle:", angle)
                if distance < 300:
                    motors.stop()
                    isDriving = False
            motors.move_forward()
        motors.turn_right(0.5)
        isDriving = True


rplidar_thread = threading.Thread(target=rp_i.get_scan, daemon=True)
x=[]
y=[]
device_x=0
device_y=0
more_x=[]
more_y=[]

rplidar_thread.start()
imu.calibrate()

plt.figure(figsize=(8, 8))
plt.title('Calibrating, please wait')
plt.xlabel('X-axis (mm)')
plt.ylabel('Y-axis (mm)')
plt.grid(True)
plt.ion()  # Turn on interactive mode
plt.title('LIDAR data')
'''
isDriving, flag2 = True, True
while flag2:
    while isDriving:
        rp_i.get_scan()
        for angle, distance in rp_i.front_data:
            if distance < 450:
                motors.stop()
                isDriving = False
        motors.move_forward()
    motors.turn_right(0.5)
    isDriving = True
    '''
for i in range(60):

    angles_rad = np.radians((rp_i.scan_data_angles-imu.new_rotation_angle) % 360)
    more_x = rp_i.scan_data_distance * np.cos(angles_rad)
    more_y = rp_i.scan_data_distance * np.sin(angles_rad)
    plt.scatter(more_x, more_y, s=10, c='black', alpha=0.5, marker='o', edgecolors='black', linewidths=1.5)
    #x.extend(more_x)
    #y.extend(more_y)
    plt.pause(1)

plt.ioff()  # Turn off interactive mode
plt.show()

rp_i.lidar.stop()
#rp_i.lidar.set_motor_pwm(0)
rp_i.lidar.disconnect()
