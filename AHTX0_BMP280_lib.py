import time
BMP280_ADDRESS = 0x77  # or 0x76 depending on your sensor's address
BMP280_REGISTER_DIG_T1 = 0x88
BMP280_REGISTER_DIG_T2 = 0x8A
BMP280_REGISTER_DIG_T3 = 0x8C
BMP280_REGISTER_DIG_P1 = 0x8E
BMP280_REGISTER_DIG_P2 = 0x90
BMP280_REGISTER_DIG_P3 = 0x92
BMP280_REGISTER_DIG_P4 = 0x94
BMP280_REGISTER_DIG_P5 = 0x96
BMP280_REGISTER_DIG_P6 = 0x98
BMP280_REGISTER_DIG_P7 = 0x9A
BMP280_REGISTER_DIG_P8 = 0x9C
BMP280_REGISTER_DIG_P9 = 0x9E
BMP280_REGISTER_CHIPID = 0xD0
BMP280_REGISTER_CONTROL = 0xF4
BMP280_REGISTER_CONFIG = 0xF5
BMP280_REGISTER_PRESSUREDATA = 0xF7
BMP280_REGISTER_TEMPDATA = 0xFA

class BMP280:
    def __init__(self, i2c, address=BMP280_ADDRESS):
        self.i2c = i2c
        self.address = address
        self._load_calibration_data()
        self._configure_sensor()

    def _load_calibration_data(self):
        calib = self.i2c.readfrom_mem(self.address, BMP280_REGISTER_DIG_T1, 24)
        self.dig_T1 = self._get_unsigned_short(calib, 0)
        self.dig_T2 = self._get_signed_short(calib, 2)
        self.dig_T3 = self._get_signed_short(calib, 4)
        self.dig_P1 = self._get_unsigned_short(calib, 6)
        self.dig_P2 = self._get_signed_short(calib, 8)
        self.dig_P3 = self._get_signed_short(calib, 10)
        self.dig_P4 = self._get_signed_short(calib, 12)
        self.dig_P5 = self._get_signed_short(calib, 14)
        self.dig_P6 = self._get_signed_short(calib, 16)
        self.dig_P7 = self._get_signed_short(calib, 18)
        self.dig_P8 = self._get_signed_short(calib, 20)
        self.dig_P9 = self._get_signed_short(calib, 22)

    def _configure_sensor(self):
        # Control register configuration: oversampling x1
        self.i2c.writeto_mem(self.address, BMP280_REGISTER_CONTROL, bytearray([0x27]))
        # Config register configuration: set inactive duration, filter, and SPI mode
        self.i2c.writeto_mem(self.address, BMP280_REGISTER_CONFIG, bytearray([0xA0]))

    def read_temperature(self):
        raw_temp = self._read24(BMP280_REGISTER_TEMPDATA)
        raw_temp >>= 4
        var1 = ((((raw_temp >> 3) - (self.dig_T1 << 1))) * (self.dig_T2)) >> 11
        var2 = (((((raw_temp >> 4) - (self.dig_T1)) * ((raw_temp >> 4) - (self.dig_T1))) >> 12) * (self.dig_T3)) >> 14
        t_fine = var1 + var2
        temperature = (t_fine * 5 + 128) >> 8
        return temperature / 100.0

    def read_pressure(self):
        raw_temp = self._read24(BMP280_REGISTER_TEMPDATA)
        raw_temp >>= 4
        var1 = ((((raw_temp >> 3) - (self.dig_T1 << 1))) * (self.dig_T2)) >> 11
        var2 = (((((raw_temp >> 4) - (self.dig_T1)) * ((raw_temp >> 4) - (self.dig_T1))) >> 12) * (self.dig_T3)) >> 14
        t_fine = var1 + var2

        raw_press = self._read24(BMP280_REGISTER_PRESSUREDATA)
        raw_press >>= 4
        var1 = (t_fine >> 1) - 64000
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 1)
        var2 = (var2 >> 2) + (self.dig_P4 << 16)
        var1 = (((self.dig_P3 * ((var1 >> 2) * (var1 >> 2)) >> 13) >> 3) + ((((var1 >> 2)) * self.dig_P2) >> 1)) >> 18
        var1 = ((32768 + var1) * self.dig_P1) >> 15
        if var1 == 0:
            return 0
        pressure = ((1048576 - raw_press) - (var2 >> 12)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure << 1) // var1
        else:
            pressure = (pressure // var1) * 2
        var1 = (self.dig_P9 * ((pressure >> 3) * (pressure >> 3)) >> 13) >> 12
        var2 = (((pressure >> 2)) * self.dig_P8) >> 13
        pressure = pressure + ((var1 + var2 + self.dig_P7) >> 4)
        return pressure / 100.0

    def _read24(self, register):
        data = self.i2c.readfrom_mem(self.address, register, 3)
        return data[0] << 16 | data[1] << 8 | data[2]

    def _get_unsigned_short(self, data, index):
        return data[index] | (data[index + 1] << 8)

    def _get_signed_short(self, data, index):
        result = data[index] | (data[index + 1] << 8)
        if result > 32767:
            result -= 65536
        return result
    
    
class AHT20:
    AHTX0_I2CADDR_DEFAULT = 0x38 #Change depending on your sensor's address
    AHTX0_CMD_CALIBRATE = 0xE1
    AHTX0_CMD_TRIGGER = 0xAC
    AHTX0_CMD_SOFTRESET = 0xBA
    AHTX0_STATUS_BUSY = 0x80
    AHTX0_STATUS_CALIBRATED = 0x08

    def __init__(self, i2c, address=AHTX0_I2CADDR_DEFAULT):
        self.i2c = i2c
        self.address = address
        self._init_sensor()

    def _init_sensor(self):
        self._soft_reset()
        self._calibrate()

    def _soft_reset(self):
        self._write_command([self.AHTX0_CMD_SOFTRESET])
        time.sleep_ms(20)

    def _calibrate(self):
        self._write_command([self.AHTX0_CMD_CALIBRATE, 0x08, 0x00])
        time.sleep_ms(20)
        if not self._is_calibrated():
            raise RuntimeError("AHT20 calibration failed")

    def _is_calibrated(self):
        status = self._read_status()
        return (status & self.AHTX0_STATUS_CALIBRATED) != 0

    def _read_status(self):
        status = self.i2c.readfrom(self.address, 1)
        return status[0]

    def _write_command(self, command):
        self.i2c.writeto(self.address, bytes(command))

    def read(self):
        self._write_command([self.AHTX0_CMD_TRIGGER, 0x33, 0x00])
        time.sleep_ms(80)
        data = self.i2c.readfrom(self.address, 7)
        if (data[0] & self.AHTX0_STATUS_BUSY) != 0:
            raise RuntimeError("AHT20 is busy")

        raw_humidity = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4))
        raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]

        humidity = (raw_humidity / 1048576.0) * 100
        temperature = (raw_temperature / 1048576.0) * 200 - 50

        return humidity, temperature

'''
#Test Code
from machine import Pin,I2C
# Initialize I2C
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)

# Main
while True:
    try:
        humidity, temperature = AHT20(i2c).read()
        print("Temperature: {:.2f} °C".format(temperature))
        print("Humidity: {:.2f} %".format(humidity))
        #print("________________")
        temperature = BMP280(i2c).read_temperature()
        pressure = BMP280(i2c).read_pressure()
        print('Temperature: {:.2f} °C'.format(temperature))
        print('Pressure: {:.2f} hPa'.format(pressure))
        print("____________________")
        
    except:print('/nsomething go wrong \n check sensor wiring')
    
    time.sleep(2)
'''
