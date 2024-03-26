'''
ten kod korzysta z elementów zaczerpniętych z:
https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module
https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi0
'''
import smbus  # import SMBus module of I2C
import time
import numpy as np
import os

# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47



def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register default 248
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)

    # Write to akcelerometr configuration register 224
    bus.write_byte_data(Device_Address, ACCL_CONFIG, 0)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()
new_rotation_angle = 0.0
last_t = time.time()
acceleration_offsets = [0, 0, 0]
gyroscope_offsets = [0, 0, 0]
acc_div = 16384.0
gyro_div = 131.0


def calibrate(t=0.01, samples=1000):

    global acceleration_offsets
    global gyroscope_offsets
    global last_t
    print("Calculating sensor offsets.")
    print("Calibration takes ", samples * t, " seconds")
    for _ in range(samples):
        raw_acc_data = np.array([read_raw_data(ACCEL_XOUT_H), read_raw_data(ACCEL_YOUT_H), read_raw_data(ACCEL_ZOUT_H)])
        acc_data = raw_acc_data / acc_div

        raw_gyro_data = np.array([read_raw_data(GYRO_XOUT_H), read_raw_data(GYRO_YOUT_H), read_raw_data(GYRO_ZOUT_H)])
        gyro_data = raw_gyro_data / gyro_div

        acceleration_offsets = [offset + accel for offset, accel in zip(acceleration_offsets, acc_data)]
        gyroscope_offsets = [offset + gyro for offset, gyro in zip(gyroscope_offsets, gyro_data)]

        time.sleep(t)

    # Calculate average offsets
    acceleration_offsets = [offset / samples for offset in acceleration_offsets]
    gyroscope_offsets = [offset / samples for offset in gyroscope_offsets]

    print("Acceleration Offsets:", acceleration_offsets)
    print("Gyroscope Offsets:", gyroscope_offsets)
    print("Reading Data of Gyroscope and Accelerometer")

    last_t = time.time()


def update_gyro():

    global new_rotation_angle
    global last_t

    raw_acc_data = np.array([read_raw_data(ACCEL_XOUT_H), read_raw_data(ACCEL_YOUT_H), read_raw_data(ACCEL_ZOUT_H)])
    raw_gyro_data = np.array([read_raw_data(GYRO_XOUT_H), read_raw_data(GYRO_YOUT_H), read_raw_data(GYRO_ZOUT_H)])

    acc_data = (raw_acc_data[2] / acc_div) * 9.807
    acc_data -= acceleration_offsets[2]
    gyro_data = raw_gyro_data[0] / gyro_div
    gyro_data -= gyroscope_offsets[0]

    t = time.time()
    dt = t - last_t
    # Integrate angular velocity to get rotation angle
    new_rotation_angle += gyro_data * dt
    # print(acc_data)
    print(new_rotation_angle)
    last_t = t

    # a1 = acc_data
    # v1 = v0+a1*t
    # sredniav = (v1+v0)/2
    # poz1 = poz0 + sredniav * t
    # print(f"{poz1}")
    # time.sleep(t)
    # v0 = v1
    # poz0 = poz1

    # v1 = v0 + acc_data * t
    # poz0 = v1 * t + 0.5 * acc_data * t ** 2
    # poz1 += poz0
    # print(f"{poz1}")
    # time.sleep(t)
    # v0 = v1

    # a1 = acc_data
    # v1 = v0 + (a1 + a0)*t/2
    # poz1 = poz0 + (v1 + v0)*t/2
    # print(f"{poz1}")
    # a0 = a1
    # v0 = v1
    # poz0 = poz1


    #acc = acc_data
    #vel += acc * dt
    #pos += vel * dt
    #print(f"{dt:.6f}")



''' aplikowanie kalibracji na wszystkie osie, zostawie to ale do ograniczenia obliczeń będe sprawdzał tylko normalnie oś X zyroskopu (skręt w lewo/ w prawo) oraz oś Z akcelerometru (przód/tył)
    while True:
        #default acc_div = 16384.0 gyro_div 131.0
        # Accelerometer and Gyro raw values
        raw_acc_data = np.array([read_raw_data(ACCEL_XOUT_H), read_raw_data(ACCEL_YOUT_H), read_raw_data(ACCEL_ZOUT_H)])
        raw_gyro_data = np.array([read_raw_data(GYRO_XOUT_H), read_raw_data(GYRO_YOUT_H), read_raw_data(GYRO_ZOUT_H)])

        acc_data = raw_acc_data/acc_div
        acc_data -= acceleration_offsets
        # Ax = raw_acc_data[0] / 16384.0 Ay = raw_acc_data[1] / 16384.0 Az = raw_acc_data[2] / 16384.0

        gyro_data = raw_gyro_data/gyro_div
        gyro_data -= gyroscope_offsets
        # Gx = raw_gyro_data[0] / 131.0 Gy = raw_gyro_data[1] / 131.0 Gz = raw_gyro_data[2] / 131.0

        #print("Gx=%.5f" % gyro_data[0], u'\u00b0' + "/s", "\tGy=%.5f" % gyro_data[1], u'\u00b0' + "/s", "\tGz=%.5f" % gyro_data[2], u'\u00b0' + "/s")
        #print("Ax=%.5f g" % acc_data[0], "\tAy=%.5f g" % acc_data[1], "\tAz=%.5f g" % acc_data[2])
        time.sleep(t)

'''