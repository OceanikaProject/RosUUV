#!/usr/bin/env python
from abc import abstractmethod
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import rospy
import smbus
from MadgwickAHRS import MadgwickAHRS
from configparser import ConfigParser
import os
import numpy as np


class ConfigError(Exception):
    """
    Exception called when sensor is configured incorrectly
    """

    def __init__(self, allowed_values):
        description = "Not a valid config. Value must be in %s" % allowed_values
        super().__init__(description)


class Sensor:

    @abstractmethod
    def startup():
        pass

    @abstractmethod
    def get_raw_data():
        pass

    @staticmethod
    def get_value(low, high):
        """
        Converting bit value of two 8-bit registers to one
        """
        value = (high << 8) | low

        if (value > 32768):
            value = value - 65536
        return value


class Magnetometer(Sensor):

    def __init__(self):
        self._mx = 0
        self._my = 0
        self._mz = 0
        self._magnetometer_range = 0

    def setM(self, mx, my, mz):
        self._mx = mx
        self._my = my
        self._mz = mz

    def set_magnetometer_range(self, sensor_range):
        self._magnetometer_range = sensor_range

    def getM(self):
        return self._mx, self._my, self._mz

    def get_magnetometer_range(self):
        return self._magnetometer_range


class Accelerometer(Sensor):

    def __init__(self):
        self._ax = 0
        self._ay = 0
        self._az = 0
        self._accelerometer_range = 0

    def setA(self, ax, ay, az):
        self._ax = ax
        self._ay = ay
        self._az = az

    def set_accelerometer_range(self, sensor_range):
        self._accelerometer_range = sensor_range

    def getA(self):
        return self._ax, self._ay, self._az

    def get_accelerometer_range(self):
        return self._accelerometer_range


class Gyroscope(Sensor):

    def __init__(self):
        self._gx = 0
        self._gy = 0
        self._gz = 0
        self._gyroscope_range

    def setG(self, gx, gy, gz):
        self._gx = gx
        self._gy = gy
        self._gz = gz

    def set_gyroscope_range(self, sensor_range):
        self._gyroscope_range = sensor_range

    def getG(self):
        return self._gx, self._gy, self._gz

    def get_gyroscope_range(self):
        return self._gyroscope_range


class Barometer(Sensor):
    def __init__(self):
        self._pressure = 0

    def setP(self, p):
        self._pressure = p

    def getP(self):
        return self._pressure


class MPU6050(Accelerometer, Gyroscope):
    PWR_M = 0x6B
    DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_EN = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40
    TEMP_OUT_H = 0x41
    TEMP_OUT_L = 0x42
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48

    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    MPU6050_ADDR = 0x68

    def __init__(self, bus, accel_range=2, gyro_range=250):

        allowed_accel_range = (2, 4, 8, 16)
        allowed_gyro_range = (250, 500, 1000, 2000)

        if accel_range not in allowed_accel_range:
            raise ConfigError(allowed_accel_range)
        if gyro_range not in allowed_gyro_range:
            raise ConfigError(allowed_gyro_range)

        if accel_range == 2:
            self.accel_config = self.ACCEL_RANGE_2G
        elif accel_range == 4:
            self.accel_config = self.ACCEL_RANGE_4G
        elif accel_range == 8:
            self.accel_config = self.ACCEL_RANGE_8G
        elif accel_range == 16:
            self.accel_config = self.ACCEL_RANGE_16G

        if gyro_range == 250:
            self.gyro_config = self.GYRO_RANGE_250DEG
        elif accel_range == 500:
            self.gyro_config = self.GYRO_RANGE_500DEG
        elif accel_range == 1000:
            self.gyro_config = self.GYRO_RANGE_1000DEG
        elif accel_range == 2000:
            self.gyro_config = self.GYRO_RANGE_2000DEG

        self.bus = bus
        self.set_accelerometer_range(float(accel_range))
        self.set_gyroscope_range(float(gyro_range))

    def startup(self):
        """
        Configuring Sensor with writing values to configure registers
        """
        self.bus.write_byte_data(self.MPU6050_ADDR, self.DIV, 7)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_M, 1)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, self.gyro_config)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, self.accel_config)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.INT_EN, 1)
        rospy.sleep(1)

    def read_status(self):
        """
        Read status register of sensor
        """
        return self.bus.read_byte_data(self.MPU6050_ADDR, self.REG_STATUS)

    def get_raw_data(self, calibrated=False):
        """
        reading data registers with values which provided by sensor
        param calibrated: if flag is on, apply calibration params to calculate values
        return:
            ax, ay, az - the projections of the gravity vector onto the axis x, y, z
            gx, gy, gz - angles of rotation about the axis x, y, z
        """
        buf = self.bus.read_i2c_block_data(self.MPU6050_ADDR, self.ACCEL_XOUT_H, 14)
        ax = self.get_value(buf[1], buf[0])
        ay = self.get_value(buf[3], buf[2])
        az = self.get_value(buf[5], buf[4])

        gx = self.get_value(buf[9], buf[8])
        gy = self.get_value(buf[11], buf[10])
        gz = self.get_value(buf[13], buf[12])

        self.setA(ax, ay, az)
        self.setG(gx, gy, gz)

        return ax, ay, az, gx, gy, gz


class QMC5883(Magnetometer):
    XOUT_LSB = 0x00
    XOUT_MSB = 0x01
    YOUT_LSB = 0x02
    YOUT_MSB = 0x03
    ZOUT_LSB = 0x04
    ZOUT_MSB = 0x05
    REG_STATUS = 0x06
    TOUT_LSB = 0x07
    TOUT_MSB = 0x08
    REG_CONTROL_1 = 0x09
    REG_CONTROL_2 = 0x0A
    REG_PERIOD = 0x0B
    ADDRESS = 0x0D

    # Oversampling values for the CONFIG register
    CONFIG_OS512 = 0b00000000
    CONFIG_OS256 = 0b01000000
    CONFIG_OS128 = 0b10000000
    CONFIG_OS64 = 0b11000000

    # Range values for the CONFIG register
    CONFIG_2GAUSS = 0b00000000
    CONFIG_8GAUSS = 0b00010000

    # Rate values for the CONFIG register
    CONFIG_10HZ = 0b00000000
    CONFIG_50HZ = 0b00000100
    CONFIG_100HZ = 0b00001000
    CONFIG_200HZ = 0b00001100

    # Mode values for the CONFIG register
    CONFIG_STANDBY = 0b00000000
    CONFIG_CONT = 0b00000001

    # Mode values for the CONFIG2 register
    CONFIG2_INT_DISABLE = 0b00000001
    CONFIG2_ROL_PTR = 0b01000000
    CONFIG2_SOFT_RST = 0b10000000

    # INI_FILE = Path(__file__).resolve().parent / 'settings' / 'QMC5883.ini'

    def __init__(self, bus, oversampling=512, range=2, rate=100, mode=1):

        allowed_oversampling = (512, 256, 128, 64)
        allowed_range = (2, 8)
        allowed_rate = (10, 50, 100, 200)
        allowed_mode = (0, 1)

        if oversampling not in allowed_oversampling:
            raise ConfigError(allowed_oversampling)
        if range not in allowed_range:
            raise ConfigError(allowed_range)
        if rate not in allowed_rate:
            raise ConfigError(allowed_rate)
        if mode not in allowed_mode:
            raise ConfigError(allowed_mode)

        if oversampling == 512:
            self.oversampling_config = QMC5883.CONFIG_OS512
        elif oversampling == 256:
            self.oversampling_config = QMC5883.CONFIG_OS256
        elif oversampling == 128:
            self.oversampling_config = QMC5883.CONFIG_OS128
        elif oversampling == 64:
            self.oversampling_config = QMC5883.CONFIG_OS64

        if range == 2:
            self.range_config = QMC5883.CONFIG_2GAUSS
        elif range == 8:
            self.range_config = QMC5883.CONFIG_8GAUSS

        if rate == 10:
            self.rate_config = QMC5883.CONFIG_10HZ
        elif rate == 50:
            self.rate_config = QMC5883.CONFIG_50HZ
        elif rate == 100:
            self.rate_config = QMC5883.CONFIG_100HZ
        elif rate == 200:
            self.rate_config = QMC5883.CONFIG_200HZ

        if mode == 0:
            self.mode_config = QMC5883.CONFIG_STANDBY
        elif mode == 1:
            self.mode_config = QMC5883.CONFIG_CONT

        self.oversampling = oversampling
        self.range = range
        self.rate = rate
        self.mode = mode

        self.bus = bus
        self.set_magnetometer_range(float(range))

    def startup(self):
        """
        Configuring Sensor with writing values to configure registers
        """
        self.bus.write_byte_data(QMC5883.ADDRESS, QMC5883.REG_CONTROL_1,
                                 self.oversampling_config | self.range_config | self.rate_config | self.mode_config)
        self.bus.write_byte_data(QMC5883.ADDRESS, QMC5883.REG_CONTROL_2, QMC5883.CONFIG2_ROL_PTR)
        self.bus.write_byte_data(QMC5883.ADDRESS, QMC5883.REG_PERIOD, 0x01)
        rospy.sleep(1)

    def read_status(self):
        """
        Read status register of sensor
        """
        return self.bus.read_byte_data(QMC5883.ADDRESS, self.REG_STATUS)

    def get_raw_data(self):
        """
        reading data registers with values which provided by sensor
        param calibrated: if flag is on, apply calibration params to calculate values
        return:
            x, y, z - the projections of the magnetic vector onto the axis x, y, z
        """
        buf = self.bus.read_i2c_block_data(QMC5883.ADDRESS, self.XOUT_LSB, 6)
        x = Sensor.get_value(buf[0], buf[1])
        y = Sensor.get_value(buf[2], buf[3])
        z = Sensor.get_value(buf[4], buf[5])

        self.setM(x, y, z)

        return x, y, z


class MS5837_30BA(Barometer):
    ADC_READ = 0x00
    RESET = 0x1E
    CONVERT_D1_OSR256 = 0x40
    CONVERT_D1_OSR512 = 0x42
    CONVERT_D1_OSR1024 = 0x44
    CONVERT_D1_OSR2048 = 0x46
    CONVERT_D1_OSR4096 = 0x48
    CONVERT_D1_OSR8192 = 0x4A
    CONVERT_D2_OSR256 = 0x50
    CONVERT_D2_OSR512 = 0x52
    CONVERT_D2_OSR1024 = 0x54
    CONVERT_D2_OSR2048 = 0x56
    CONVERT_D2_OSR4096 = 0x58
    CONVERT_D2_OSR8192 = 0x5A
    PROM_READ = 0xA0

    MS5837_30BA_ADDRESS = 0x76

    def __init__(self, bus):
        self.bus = bus
        self.depth_mean = []

    def startup(self):
        self.bus.write_byte(self.MS5837_30BA_ADDRESS, self.RESET)
        rospy.sleep(.01)
        print('ok')
        self.C = []
        for i in range(7):
            c = self.bus.read_word_data(self.MS5837_30BA_ADDRESS, self.PROM_READ + 2 * i)
            c = ((c & 0xFF) << 8) | (c >> 8)
            self.C.append(c)
        crc = (self.C[0] & 0xF000) >> 12
        if crc != self._crc4(self.C):
            return
        # print(self.l, crc, crc == self.crc4(self.l))

    def _crc4(self, n_prom):
        n_rem = 0

        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom.append(0)

        for i in range(16):
            if i % 2 == 1:
                n_rem ^= ((n_prom[i >> 1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i >> 1] >> 8)

            for n_bit in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)

        n_rem = ((n_rem >> 12) & 0x000F)

        self.n_prom = n_prom
        self.n_rem = n_rem

        return n_rem ^ 0x00

    def getPressure(self, conversion=100):
        return self.getP() * conversion

    def getDepth(self):
        return round((self.getPressure() - 101325) / (9.80665 * 997), 2)

    def getAltitude(self):
        return round((1 - pow((self.getPressure() / 101325), .190295)) * 145366.45 * .3048, 2)

    def get_raw_data(self):
        self.bus.write_byte(self.MS5837_30BA_ADDRESS, self.CONVERT_D1_OSR8192)
        rospy.sleep(2.5e-6 * 2 ** (8 + 5))
        pdata = self.bus.read_i2c_block_data(self.MS5837_30BA_ADDRESS, self.ADC_READ, 3)
        D1 = pdata[0] << 16 | pdata[1] << 8 | pdata[2]
        self.bus.write_byte(self.MS5837_30BA_ADDRESS, self.CONVERT_D2_OSR8192)
        rospy.sleep(2.5e-6 * 2 ** (8 + 5))
        tdata = self.bus.read_i2c_block_data(self.MS5837_30BA_ADDRESS, self.ADC_READ, 3)
        D2 = tdata[0] << 16 | tdata[1] << 8 | tdata[2]
        self.calculate(D1, D2)

    def setT(self, T):
        self.T = T

    def getT(self):
        return self.T

    def calculate(self, D1, D2):
        OFFi = 0
        SENSi = 0
        Ti = 0

        dT = D2 - self.C[5] * 256

        SENS = self.C[1] * 32768 + (self.C[3] * dT) / 256
        OFF = self.C[2] * 65536 + (self.C[4] * dT) / 128
        pr = (D1 * SENS / (2097152) - OFF) / (8192)

        temp = 2000 + dT * self.C[6] / 8388608

        if temp / 100 < 20:
            Ti = (3 * dT * dT) / 8589934592
            OFFi = (3 * (temp - 2000) * (temp - 2000)) / 2
            SENSi = (5 * (temp - 2000) * (temp - 2000)) / 8
            if temp / 100 < -15:
                OFFi = OFFi + 7 * (temp + 1500) * (temp + 1500)
                SENSi = SENSi + 4 * (temp + 1500) * (temp + 1500)
        else:
            Ti = 2 * (dT * dT) / 137438953472
            OFFi = (temp - 2000) * (temp - 2000) / 16
            SENSi = 0

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi

        temp = (temp - Ti) / 100
        pr = ((D1 * SENS2 / 2097152 - OFF2) / 8192) / 10.0
        self.setP(pr)
        self.setT(temp)


class InertialNavigationSystem:

    def __init__(self):
        self.sensors = {}

        self._ax = 0
        self._ay = 0
        self._az = 0
        self._gx = 0
        self._gy = 0
        self._gz = 0
        self._mx = 0
        self._my = 0
        self._mz = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.xpos = 0
        self.ypos = 0
        self.depth = 0

        self.madgwick_filter = MadgwickAHRS()

        self.config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/config.ini')
        self.configparser = ConfigParser()
        self.configparser.read(self.config_path)

    def startup_sensors(self, sensors):
        for sensor in sensors:
            if isinstance(sensor, Accelerometer):
                self.sensors['accelerometer'] = sensor
            if isinstance(sensor, Gyroscope):
                self.sensors['gyroscope'] = sensor
            if isinstance(sensor, Magnetometer):
                self.sensors['magnetometer'] = sensor
            if isinstance(sensor, Barometer):
                self.sensors['barometer'] = sensor
            sensor.startup()
        if not self.sensors['accelerometer'] or not self.sensors['gyroscope']:
            raise NotImplementedError
        if not self.sensors['magnetometer']:
            raise Warning

    def set_real_sensor_values(self):
        """
        convert raw binary sensor values into decimal unit system
        """

        def convert(data, sensor_range):
            for index in range(len(data)):
                data[index] = data[index] / 32768.0 * sensor_range
            return data

        sensors = set(val for val in self.sensors.values())
        for sensor in sensors:
            sensor.get_raw_data()
        ax, ay, az = self.sensors['accelerometer'].getA()
        gx, gy, gz = self.sensors['gyroscope'].getG()
        mx, my, mz = self.sensors['magnetometer'].getM()
        pressure = self.sensors['barometer'].getP()
        self.depth = self.sensors['barometer'].getDepth()
        self._ax, self._ay, self._az = convert([ax, ay, az], self.sensors['accelerometer'].get_accelerometer_range())
        self._gx, self._gy, self._gz = convert([gx, gy, gz], self.sensors['gyroscope'].get_gyroscope_range())
        # self._mx, self._my, self._mz = mx, my, mz
        self._mx, self._my, self._mz = convert([mx, my, mz], self.sensors['magnetometer'].get_magnetometer_range())
        # print(
        #     round(self._ax, 2),round(self._ay, 2),round(self._az, 2),
        #     round(self._gx, 2),round(self._gy, 2),round(self._gz, 2),
        #     # self._mx,self._my,self._mz
        # )

    def get_v(self):
        return self._ax, self._ay, self._az, self._gx, self._gy, self._gz

    def __calibrate_magnetometer(self, rounds):
        """
        Magnetometer calibration
        rounds: number of read cycles
        """
        xmin, ymin, zmin = 32767, 32767, 32767
        xmax, ymax, zmax = -32768, -32768, -32768
        print("Rotate compass")
        rospy.sleep(2)
        for _ in range(rounds):
            self.set_real_sensor_values()
            print(self._mx, self._my, self._mz)

            xmin = min(xmin, self._mx)
            ymin = min(ymin, self._my)
            zmin = min(zmin, self._mz)

            xmax = max(xmax, self._mx)
            ymax = max(ymax, self._my)
            zmax = max(zmax, self._mz)
            rospy.sleep(0.005)

        x_offset = (xmax + xmin) / 2
        y_offset = (ymax + ymin) / 2
        z_offset = (zmax + zmin) / 2

        chord_x = (xmax - xmin) / 2.0
        chord_y = (ymax - ymin) / 2.0
        chord_z = (zmax - zmin) / 2.0

        chord_average = (chord_x + chord_y + chord_z) / 3.0

        x_scale = chord_average / chord_x
        y_scale = chord_average / chord_y
        z_scale = chord_average / chord_z

        self.configparser.set("Magnetometer", "mxoffset", str(x_offset))
        self.configparser.set("Magnetometer", "myoffset", str(y_offset))
        self.configparser.set("Magnetometer", "mzoffset", str(z_offset))
        self.configparser.set("Magnetometer", "mxscale", str(x_scale))
        self.configparser.set("Magnetometer", "myscale", str(y_scale))
        self.configparser.set("Magnetometer", "mzscale", str(z_scale))

    def __calibrate_gyroscope_and_accelerometer(self, rounds):
        """
        Gyroscope and accelerometer calibration
        rounds: number of read cycles
        """
        gxcal, gycal, gzcal = 0, 0, 0
        axcal, aycal, azcal = 0, 0, 0
        print("Keep gyroscope steady")
        rospy.sleep(5)
        for _ in range(rounds):
            self.set_real_sensor_values()

            print(round(self._ax, 2), round(self._ay, 2), round(self._az, 2), round(self._gx, 2), round(self._gy, 2),
                  round(self._gz, 2))

            gxcal += self._gx
            gycal += self._gy
            gzcal += self._gz
            axcal += self._ax
            aycal += self._ay
            azcal += self._az - 1

            rospy.sleep(.005)

        gxcal /= rounds
        gycal /= rounds
        gzcal /= rounds
        axcal /= rounds
        aycal /= rounds
        azcal /= rounds

        self.configparser.set("Gyroscope", "gxoffset", str(gxcal))
        self.configparser.set("Gyroscope", "gyoffset", str(gycal))
        self.configparser.set("Gyroscope", "gzoffset", str(gzcal))
        self.configparser.set("Accelerometer", "axoffset", str(axcal))
        self.configparser.set("Accelerometer", "ayoffset", str(aycal))
        self.configparser.set("Accelerometer", "azoffset", str(azcal))

    def __compute_std(self, rounds=10000):
        """
        Collection of readings using calibration values with further calculation of standart deviations for Kalman filter
        rounds: number of read cycles
        """
        storage = {
            'ax': [],
            'ay': [],
            'az': [],
            'gx': [],
            'gy': [],
            'gz': [],
            'mx': [],
            'my': [],
            'mz': []
        }
        for _ in range(rounds):
            self.set_real_sensor_values()
            storage['ax'].append(self._ax)
            storage['ay'].append(self._ay)
            storage['az'].append(self._az)
            storage['gx'].append(self._gx)
            storage['gy'].append(self._gy)
            storage['gz'].append(self._gz)
            storage['mx'].append(self._mx)
            storage['my'].append(self._my)
            storage['mz'].append(self._mz)
        axstd = np.std(storage['ax'])
        aystd = np.std(storage['ay'])
        azstd = np.std(storage['az'])
        gxstd = np.std(storage['gx'])
        gystd = np.std(storage['gy'])
        gzstd = np.std(storage['gz'])
        mxstd = np.std(storage['mx'])
        mystd = np.std(storage['my'])
        mzstd = np.std(storage['mz'])

        self.configparser.set("Accelerometer", "axstd", str(axstd))
        self.configparser.set("Accelerometer", "aystd", str(aystd))
        self.configparser.set("Accelerometer", "azstd", str(azstd))
        self.configparser.set("Gyroscope", "gxstd", str(gxstd))
        self.configparser.set("Gyroscope", "gystd", str(gystd))
        self.configparser.set("Gyroscope", "gzstd", str(gzstd))
        self.configparser.set("Magnetometer", "mxstd", str(mxstd))
        self.configparser.set("Magnetometer", "mystd", str(mystd))
        self.configparser.set("Magnetometer", "mzstd", str(mzstd))

    def set_configured_values(self):
        """
        reading calibration values and calculating sensor values using them
        """
        self.set_real_sensor_values()

        self._gx = self._gx - self.configparser.getfloat("Gyroscope", "gxoffset")
        self._gy = self._gy - self.configparser.getfloat("Gyroscope", "gyoffset")
        self._gz = self._gz - self.configparser.getfloat("Gyroscope", "gzoffset")
        self._mx = (self._mx - self.configparser.getfloat("Magnetometer", "mxoffset")) * self.configparser.getfloat(
            "Magnetometer", "mxscale")
        self._my = (self._my - self.configparser.getfloat("Magnetometer", "myoffset")) * self.configparser.getfloat(
            "Magnetometer", "myscale")
        self._mz = (self._mz - self.configparser.getfloat("Magnetometer", "mzoffset")) * self.configparser.getfloat(
            "Magnetometer", "mzscale")
        self._gx = -gy
        self._ax = self._ax - self.configparser.getfloat("Accelerometer", "axoffset")
        self._ay = self._ay - self.configparser.getfloat("Accelerometer", "ayoffset")
        self._az = self._az - self.configparser.getfloat("Accelerometer", "azoffset")

    def update_state(self, dt):
        """
        calculating of Euler angles
        return: list of Euler anlges
        """
        # self.set_configured_values()
        self.set_real_sensor_values()
        # print(round(self._ax, 2), round(self._ay, 2), round(self._az, 2), end=' ')
        # vals = self.madgwick_filter.update([self._gx, self._gy, self._gz], [self._ax, self._ay, self._az], [self._mx, self._my, self._mz])
        # angles, a = vals[:3], vals[3:]
        # angles = [rad * 180 / 3.1415926535 for rad in angles]
        # self.roll, self.pitch, self.yaw = angles
        # self._ax, self._ay, self._az = a

        self.q = self.madgwick_filter.update([self._gx, self._gy, self._gz], [self._ax, self._ay, self._az],
                                             [self._mx, self._my, self._mz])

        # self.kf.go(dt, self._ax, self._ay, self._az)

        # self._ax *= 9.80665
        # self._ay *= 9.80665
        # self._az *= 9.80665
        # if yaw < 0: yaw += 360

        # print(round(self._ax, 2), round(self._ay, 2), round(self._az, 2))
        # if not -0.1 < self._ax < 0.1:
        #     self.vx = round(self.vx + self._ax * dt, 2)
        # if not -0.1 < self._ay < 0.1:
        #     self.vy = round(self.vy + self._ay * dt, 2)
        # if not -0.1 < self._az < 0.1:
        #     self.vz = round(self.vz + self._az * dt, 2)

        # self.xpos = round(self.xpos + self.vx * dt, 2)
        # self.ypos = round(self.ypos + self.vy * dt, 2)
        # self.depth = round(self.depth + self.vz * dt, 2)
        # self.roll = round(self.roll, 2)
        # self.pitch = round(self.pitch, 2)
        # self.yaw = round(self.yaw, 2)

    @property
    def state(self):
        # return [self.roll, self.pitch, self.yaw]
        return self.q, self.depth

    def configure_system(self):
        """
        System setup
        """
        self.__calibrate_magnetometer(400)
        self.__calibrate_gyroscope_and_accelerometer(400)
        self.__compute_std(1000)
        with open(self.config_path, 'w+') as config_params:
            self.configparser.write(config_params)


def talker():
    # pub = rospy.Publisher('talker_topic', Float32MultiArray, queue_size=10)
    pub = rospy.Publisher('talker_topic', PoseStamped, queue_size=10)
    imu_pub = rospy.Publisher('imu_pub', Imu, queue_size=5)
    rospy.init_node('talker_node')
    bus = smbus.SMBus(1)
    SENSORS = [MPU6050(bus), QMC5883(bus), MS5837_30BA(bus)]
    system = InertialNavigationSystem()
    system.startup_sensors(SENSORS)
    # imu = MPU6050(bus)
    # compass = QMC5883(bus)
    # press = MS5837_30BA(bus)
    # imu.startup()
    # compass.startup()
    # press.startup()
    rate = rospy.Rate(1000)
    rospy.loginfo("Talker started")
    # system.configure_system()
    while not rospy.is_shutdown():
        system.update_state(1)
        # ax, ay, az, gx, gy, gz = imu.get_raw_data()
        # mx, my, mz = compass.get_raw_data()

        # press.get_raw_data()
        # alt = press.getAltitude()
        q, z = system.state
        # msg = "(%f, %f, %f)|(%f, %f, %f)|(%f, %f, %f)" % (state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8])
        # msg = Float32MultiArray(data=state)
        msg = PoseStamped()
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]

        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = z

        imu_msg = Imu()
        imu_msg.orientation.w = q[0]
        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]

        pub.publish(msg)
        imu_pub.publish(imu_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass