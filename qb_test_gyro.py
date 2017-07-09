from Adafruit_I2C import Adafruit_I2C

i2c = Adafruit_I2C(0x6a,2)

reg = 0

print i2c.readS16(0x4819c000)

print reg


