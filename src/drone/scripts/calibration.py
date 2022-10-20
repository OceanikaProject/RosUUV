#!/usr/bin/env python3
from abc import abstractmethod
import smbus
import os
import yaml
import rospy


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

        self.config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/calibration.yaml')
        

    def startup_sensors(self, sensors):
        for sensor in sensors:
            if isinstance(sensor, Accelerometer):
                self.sensors['accelerometer'] = sensor
            if isinstance(sensor, Gyroscope):
                self.sensors['gyroscope'] = sensor
            if isinstance(sensor, Magnetometer):
                self.sensors['magnetometer'] = sensor
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
        self._ax, self._ay, self._az = convert([ax, ay, az], self.sensors['accelerometer'].get_accelerometer_range())
        self._gx, self._gy, self._gz = convert([gx, gy, gz], self.sensors['gyroscope'].get_gyroscope_range())
        self._mx, self._my, self._mz = convert([mx, my, mz], self.sensors['magnetometer'].get_magnetometer_range())

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
        rospy.sleep(4)
        for _ in range(rounds):
            self.set_real_sensor_values()
            # self._mx, self._my, self._mz = -self._mx, -self._my, -self._mz
            print(self._mx, self._my, self._mz)

            xmin = min(xmin, self._mx)
            ymin = min(ymin, self._my)
            zmin = min(zmin, self._mz)

            xmax = max(xmax, self._mx)
            ymax = max(ymax, self._my)
            zmax = max(zmax, self._mz)
            rospy.sleep(0.05)

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

        with open(self.config_path, 'r') as f:
            conf = yaml.safe_load(f)
        
        conf["Magnetometer"]["mxoffset"] = x_offset
        conf["Magnetometer"]["myoffset"] = y_offset
        conf["Magnetometer"]["mzoffset"] = z_offset
        conf["Magnetometer"]["mxscale"] = x_scale
        conf["Magnetometer"]["myscale"] = y_scale
        conf["Magnetometer"]["mzscale"] = z_scale

        with open(self.config_path, 'w') as f:
            yaml.dump(conf, f, default_flow_style=False)

    def __calibrate_gyroscope_and_accelerometer(self, rounds):
        """
        Gyroscope and accelerometer calibration
        rounds: number of read cycles
        """
        gxcal, gycal, gzcal = 0, 0, 0
        axcal, aycal, azcal = 0, 0, 0
        print("Keep gyroscope steady")
        rospy.sleep(6)
        for _ in range(rounds):
            self.set_real_sensor_values()
            # self._ax, self._ay, self._az = -self._ax, -self._ay, -self._az
            # self._gx, self._gy, self._gz = -self._gx, -self._gy, -self._gz

            print(round(self._ax, 2), round(self._ay, 2), round(self._az, 2), round(self._gx, 2), round(self._gy, 2),
                  round(self._gz, 2))

            gxcal += self._gx
            gycal += self._gy
            gzcal += self._gz
            axcal += self._ax
            aycal += self._ay
            azcal += self._az - 1

            rospy.sleep(.05)

        gxcal /= rounds
        gycal /= rounds
        gzcal /= rounds
        axcal /= rounds
        aycal /= rounds
        azcal /= rounds

        with open(self.config_path, 'r') as f:
            conf = yaml.safe_load(f)

        conf["Gyroscope"]["gxoffset"] = gxcal
        conf["Gyroscope"]["gzoffset"] = gycal
        conf["Gyroscope"]["gyoffset"] = gzcal
        conf["Accelerometer"]["axoffset"] = axcal
        conf["Accelerometer"]["ayoffset"] = aycal
        conf["Accelerometer"]["azoffset"] = azcal

        with open(self.config_path, 'w') as f:
            yaml.dump(conf, f, default_flow_style=False)

    def configure_system(self):
        """
        System setup
        """
        self.__calibrate_magnetometer(600)
        self.__calibrate_gyroscope_and_accelerometer(600)
        # self.__compute_std(1000)
        # with open(self.config_path, 'w+') as config_params:
        #     self.configparser.write(config_params)



if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)
        sys = InertialNavigationSystem()
        sys.startup_sensors([MPU6050(bus), QMC5883(bus)])
        sys.configure_system()
    except rospy.ROSInterruptException:
        pass