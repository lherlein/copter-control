from machine import I2C, Pin
from constants_imu import *
import time

class AltIMU10:

    # Output registers used by the gyroscope
    gyro_registers = [
        LSM6DS33_OUTX_L_G,  # low byte of X value
        LSM6DS33_OUTX_H_G,  # high byte of X value
        LSM6DS33_OUTY_L_G,  # low byte of Y value
        LSM6DS33_OUTY_H_G,  # high byte of Y value
        LSM6DS33_OUTZ_L_G,  # low byte of Z value
        LSM6DS33_OUTZ_H_G,  # high byte of Z value
    ]

    # Output registers used by the accelerometer
    accel_registers = [
        LSM6DS33_OUTX_L_XL,  # low byte of X value
        LSM6DS33_OUTX_H_XL,  # high byte of X value
        LSM6DS33_OUTY_L_XL,  # low byte of Y value
        LSM6DS33_OUTY_H_XL,  # high byte of Y value
        LSM6DS33_OUTZ_L_XL,  # low byte of Z value
        LSM6DS33_OUTZ_H_XL,  # high byte of Z value
    ]

    # Output registers used by the barometer sensor
    barometer_registers = [
        LPS25H_PRESS_OUT_XL,  # lowest byte of pressure value
        LPS25H_PRESS_OUT_L,   # low byte of pressure value
        LPS25H_PRESS_OUT_H,   # high byte of pressure value
    ]

    # Output registers used by the magnetometer
    magnetometer_registers = [
        LIS3MDL_OUT_X_L,  # low byte of X value
        LIS3MDL_OUT_X_H,  # high byte of X value
        LIS3MDL_OUT_Y_L,  # low byte of Y value
        LIS3MDL_OUT_Y_H,  # high byte of Y value
        LIS3MDL_OUT_Z_L,  # low byte of Z value
        LIS3MDL_OUT_Z_H,  # high byte of Z value
    ]

    ACC_GYRO_ADDR = 0x6b
    MAG_ADDR = 0x1e
    BARO_ADDR = 0x5d

    def __init__(self, i2c):
        self.i2c = i2c
        self._initialize_sensors()
        
        self.accel_angle_cal = [0, 0]
        self.gyro_cal = [0, 0, 0]
        
        self.is_gyro_calibrated = False
        #self.calibrate()

    def _initialize_sensors(self):
        # Initialize accelerometer and gyroscope
        self._write_register(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, bytes(0x58))  # CTRL_REG1_G (power up gyroscope)
        self._write_register(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x58)  # CTRL_REG6_XL (power up accelerometer)
        
        # Initialize magnetometer
        # Disable magnetometer and temperature sensor first
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x00)
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)

        # Enable device in continuous conversion mode
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)

        # Initial value for CTRL_REG1
        ctrl_reg1 = 0x00

        # Ultra-high-performance mode for X and Y
        # Output data rate 10Hz
        # binary value -> 01110000b, hex value -> 0x70
        ctrl_reg1 += 0x70

        # +/- 4 gauss full scale
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x00)

        # Ultra-high-performance mode for Z
        # binary value -> 00001100b, hex value -> 0x0c
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, 0x0c)

        # Write calculated value to the CTRL_REG1 register
        self._write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, ctrl_reg1)
        
        # Initialize barometric pressure sensor
        self._write_register(LPS25H_ADDR, LPS25H_CTRL_REG1, 0x00)  # CTRL_MEAS (configure barometer)
        self._write_register(LPS25H_ADDR, LPS25H_CTRL_REG1, 0xb0)  # CTRL_REG1 (power up barometer)
        
    def _read_register(self, addr, reg, nbytes=1):
        return self.i2c.readfrom_mem(addr, reg, nbytes)

    def _write_register(self, addr, reg, data):
        """Write an integer value to an I2C register, dynamically determining the number of bytes needed."""
        if isinstance(data, int):
            # Ensure at least one byte is used
            data_bytes = data.to_bytes(1, 'little')
        elif isinstance(data, bytes):
            data_bytes = data
        else:
            raise ValueError("Data must be an integer or bytes-like object")

        self.i2c.writeto_mem(addr, reg, data_bytes)

    def read_accelerometer_raw(self):
        return self.read_3d_sensor(LSM6DS33_ADDR, self.accel_registers)

    def read_gyroscope_raw(self):
        sensor_data = self.read_3d_sensor(LSM6DS33_ADDR, self.gyro_registers)

        # Return the vector
        if self.is_gyro_calibrated:
            calibrated_gyro_data = sensor_data
            calibrated_gyro_data[0] -= self.gyro_cal[0]
            calibrated_gyro_data[1] -= self.gyro_cal[1]
            calibrated_gyro_data[2] -= self.gyro_cal[2]
            return calibrated_gyro_data
        else:
            return sensor_data

    def read_magnetometer_raw(self):
        return self.read_3d_sensor(LIS3MDL_ADDR, self.magnetometer_registers)

    def read_barometer_raw(self):
        return self.read_1d_sensor(LPS25H_ADDR, self.barometer_registers)

    def read_3d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 16 bit values
            of the output registers of a 3d sensor.
        """

        # Read register outputs and combine low and high byte values
        x_low = self._read_register(address, registers[0])
        x_hi = self._read_register(address, registers[1])
        y_low = self._read_register(address, registers[2])
        y_hi = self._read_register(address, registers[3])
        z_low = self._read_register(address, registers[4])
        z_hi = self._read_register(address, registers[5])

        x_val = self.combine_signed_lo_hi(x_low, x_hi)
        y_val = self.combine_signed_lo_hi(y_low, y_hi)
        z_val = self.combine_signed_lo_hi(z_low, z_hi)

        return [x_val, y_val, z_val]

    def read_1d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 24 bit values
            of the output registers of a 1d sensor.
        """

        xlo_byte = self._read_register(address, registers[0])
        lo_byte = self._read_register(address, registers[1])
        hi_byte = self._read_register(address, registers[2])

        return self.combine_signed_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)

    def combine_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to an unsigned 16 bit value. """
        # Convert bytes to integers
        if isinstance(lo_byte, bytes) and isinstance(hi_byte, bytes):
            lo_byte = int.from_bytes(lo_byte, 'little')
            hi_byte = int.from_bytes(hi_byte, 'little')
        elif isinstance(lo_byte, bytes) or isinstance(hi_byte, bytes):
            raise ValueError("Both lo_byte and hi_byte must be of the same type")
        
        # Ensure they are integers
        lo_byte = int(lo_byte)
        hi_byte = int(hi_byte)

        return (hi_byte << 8) | lo_byte

    def combine_signed_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to a signed 16 bit value. """
        combined = self.combine_lo_hi(lo_byte, hi_byte)
        return combined if combined < 32768 else (combined - 65536)

    def combine_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to an unsigned
            24 bit value.
        """
        # Convert bytes to integers
        if isinstance(lo_byte, bytes) and isinstance(hi_byte, bytes):
            lo_byte = int.from_bytes(lo_byte, 'little')
            hi_byte = int.from_bytes(hi_byte, 'little')
            xlo_byte = int.from_bytes(xlo_byte, 'little')
        elif isinstance(lo_byte, bytes) or isinstance(hi_byte, bytes):
            raise ValueError("Both lo_byte and hi_byte must be of the same type")
        
        # Ensure they are integers
        lo_byte = int(lo_byte)
        hi_byte = int(hi_byte)
        xlo_byte = int(xlo_byte)

        return (xlo_byte | lo_byte << 8 | hi_byte << 16)

    def combine_signed_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to a signed 24 bit value. """
        combined = self.combine_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)
        return combined if combined < 8388608 else (combined - 16777216)

    def calibrate(self, iterations=500):
        """ Calibrate the gyro's raw values."""
        print('Calibrating Gryo and Accelerometer...')

        for i in range(iterations):
            gyro_raw = self.read_gyroscope_raw()
            accel_angles = self.read_accelerometer_raw() # TODO - USE ANGLES HERE

            self.gyro_cal[0] += gyro_raw[0]
            self.gyro_cal[1] += gyro_raw[1]
            self.gyro_cal[2] += gyro_raw[2]

            self.accel_angle_cal[0] += accel_angles[0]
            self.accel_angle_cal[1] += accel_angles[1]

            time.sleep(0.004)

        self.gyro_cal[0] /= iterations
        self.gyro_cal[1] /= iterations
        self.gyro_cal[2] /= iterations

        self.accel_angle_cal[0] /= iterations
        self.accel_angle_cal[1] /= iterations

        self.is_gyro_calibrated = True

        print('Calibration Done')