import FaBo9Axis_MPU9250
from collections import defaultdict
from picamera import PiCamera
import time
from smbus2 import SMBus
import sys
import csv
import os
from datetime import datetime
import RPi.GPIO as GPIO

"""CSV Specification
humidity: relative humidity measured as a percentage
humidity_temp: temperature as measured by the humidity sensor in C
pressure: current air pressure in kPa
altitude: current height in m
altimeter_temp: temperature as measured by the altimeter in C
accel_x: x-direction acceleration in m/s^2
accel_y: y-direction acceleration in m/s^2
accel_z: z-direction acceleration in m/s^2
gyro_x: x-direction angular vel in rad/s
gyro_y: y-direction angular vel in rad/s
gryo_z: z-direction angular vel in rad/s
mag_x: x-direction magnetic field in uT
mag_y: y-direction magnetic field in uT
mag_z: z-direction magnetic field in uT
"""

class Sensors:
    
    def bad_sample():
        return "Measurement not present"
    
    dataDictionary = defaultdict(bad_sample)
    
    def __init__(self):
        self.data = dataDictionary
        
        
    def readSensors(self,arg):
        duration = int(arg)
        now = datetime.now()
        dt_string = now.strftime("%d_%H%M%S")
        with open(f'balloon_stats_{dt_string}.csv', 'w', newline='', buffering=1) as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.dataDictionary.keys())
            csv_writer.writeheader()
            for i in range(duration):
                self.sampleSensors(csv_writer)
                time.sleep(6.5)       # sets capture interval to 1 minute 
        return self.data   
    
    
    def sampleSensors(self,csv_writer):
        #add method to sample sensor here w/ exception safety
        #add LED status triggers here
        
        try:
            self.run_time()
        except Exception as err:
            print("time error: ", err)
            
        try:
            self.run_altimeter()
        except Exception as err:
            print("altimeter sensor error: ", err)
            
        try:
            self.run_humidity()
        except Exception as err:
            print("humidity sensor error: ", err)
        
        try:    
            self.run_imu()
        except Exception as err: 
            print("imu sensor error: ", err)
            
        csv_writer.writerow(data)
        
        try:
            self.run_camera()
        except Exception as err:
            print("camera error: ", err)
        
      
    
    def run_camera(self):
        with PiCamera() as camera:
            camera.resolution=(1920, 1080)
            camera.start_preview()
            camera.capture(f'/home/pi/pictures/picture{i}.jpeg', format='jpeg')
            camera.stop_preview()       
             
    def run_time(self):
        self.data['time'] = time.strftime("%H:%M:%S")
        
          
    def run_humidity(self):
        with SMBus(1) as bus:

            # SHT31 address, 0x44(68)
            bus.write_i2c_block_data(0x44, 0x2C, [0x06])

            time.sleep(0.5)

            # SHT31 address, 0x44(68)
            # Read data back from 0x00(00), 6 bytes
            # Temp MSB, Temp LSB, Temp CRC, Humididty MSB, Humidity LSB, Humidity CRC
            data = bus.read_i2c_block_data(0x44, 0x00, 6)

            # Convert the data
            temp = data[0] * 256 + data[1]
            cTemp = -45 + (175 * temp / 65535.0)
            # fTemp = -49 + (315 * temp / 65535.0)
            humidity = 100 * (data[3] * 256 + data[4]) / 65535.0

            #pass measured values into dataDictionary
            self.data['humidity'] = f"{humidity:.2f}"
            self.data['humidity_temp'] = f"{cTemp:.2f}"

    def run_altimeter(self):
        with SMBus(1) as bus:
            time.sleep(0.25)
            # MPL3115A2 address, 0x60(96)
            # Select control register, 0x26(38)
            #		0xB9(185)	Active mode, OSR = 128, Altimeter mode
            bus.write_byte_data(0x60, 0x26, 0xB9,True)
            # MPL3115A2 address, 0x60(96)
            # Select data configuration register, 0x13(19)
            #		0x07(07)	Data ready event enabled for altitude, pressure, temperature
            bus.write_byte_data(0x60, 0x13, 0x07,True)
            # MPL3115A2 address, 0x60(96)
            # Select control register, 0x26(38)
            #		0xB9(185)	Active mode, OSR = 128, Altimeter mode
            bus.write_byte_data(0x60, 0x26, 0xB9,True)

            time.sleep(1)

            # MPL3115A2 address, 0x60(96)
            # Read data back from 0x00(00), 6 bytes
            # status, tHeight MSB1, tHeight MSB, tHeight LSB, temp MSB, temp LSB
            data = bus.read_i2c_block_data(0x60, 0x00, 6)

            # Convert the data to 20-bits
            tHeight = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
            temp = ((data[4] * 256) + (data[5] & 0xF0)) / 16
            altitude = tHeight / 16.0
            cTemp = temp / 16.0
            # MPL3115A2 address, 0x60(96)
            # Select control register, 0x26(38)
            #		0x39(57)	Active mode, OSR = 128, Barometer mode
            bus.write_byte_data(0x60, 0x26, 0x39)

            time.sleep(1)

            # MPL3115A2 address, 0x60(96)
            # Read data back from 0x00(00), 4 bytes
            # status, pres MSB1, pres MSB, pres LSB
            data = bus.read_i2c_block_data(0x60, 0x00, 4)

            # Convert the data to 20-bits
            pres = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
            pressure = (pres / 4.0) / 1000.0

            #pass measured values into dataDictionary
            self.data['pressure'] = f'{pressure:.2f}'
            self.data['altitude'] = f'{altitude:.2f}'
            self.data['altimeter_temp'] = f'{cTemp:.2f}'
        
                 
    def run_imu(self):
        mpu9250 = FaBo9Axis_MPU9250.MPU9250()

        accel = mpu9250.readAccel()
        
        #pass measured values into dataDictionary
        self.data['accel_x'] = str(accel['x'])
        self.data['accel_y'] = str(accel['y'])
        self.data['accel_z'] = str(accel['z'])

        gyro = mpu9250.readGyro()
        self.data['gyro_x'] = str(gyro['x'])
        self.data['gyro_y'] = str(gyro['y'])
        self.data['gyro_z'] = str(gyro['z'])

        mag = mpu9250.readMagnet()
        self.data['mag_x'] = str(mag['x'])
        self.data['mag_y'] = str(mag['y'])
        self.data['mag_z'] = str(mag['z'])