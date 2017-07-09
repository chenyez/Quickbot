from Adafruit_I2C import Adafruit_I2C
from LSM9DS0 import *
import time
import numpy as np

# class to interact with Berry IMU compass and get the current heading of the robot
# the actual compass could be anything
class Compass(object):
	def __init__(self):
		# Berry IMU Specific	
		self.BIMU_mag_addr = 0x1e # both accel and compass
		self.BIMU_acc_addr = 0x1e # both accel and compass
		self.BIMU_gyro_addr = 0x6a # gyro only
		self.i2c_mag = Adafruit_I2C(self.BIMU_mag_addr,2);
		# enable all axes in normal power mode
		self.i2c_mag.write8(CTRL_REG5_XM,0xf0);
		# set continuous update, full scale
		self.i2c_mag.write8(CTRL_REG6_XM,0x60);
		self.i2c_mag.write8(CTRL_REG7_XM,0x00);
	# returns the current heading
	def get_heading(self):
		
		xl=self.i2c_mag.readU8(OUT_X_L_M);
        	xh=self.i2c_mag.readS8(OUT_X_H_M);
        	x = ((xh<<8)|xl)
        	yl=self.i2c_mag.readU8(OUT_Y_L_M);
        	yh=self.i2c_mag.readS8(OUT_Y_H_M);
        	y = ((yh<<8)|yl)
        	zl=self.i2c_mag.readU8(OUT_Z_L_M);
        	zh=self.i2c_mag.readS8(OUT_Z_H_M);
        	z = ((zh<<8)|zl)

        	heading = 180.0*np.arctan2(-y,x)/np.pi
        	#print 'heading=',heading
        	if heading < 0:
        	        heading += 360
        	#print 'heading = ',heading, 'raw mag x = ',x,' y= ',y,' z= ',z
		return heading;

if __name__ == '__main__':
	print 'heading = ',Compass().get_heading();

