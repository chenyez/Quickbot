import smbus
import time
import math
import numpy as np
from numpy import linalg as la
bus = smbus.SMBus(1)
address = 0x1e

class Compass:
	def __init__(self):
		self.write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
		self.write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
		self.write_byte(2, 0b00000000) # Continuous sampling
	
		self.scale = 0.92
		self.x_offset = -95
		self.y_offset = -104
		
	def read_byte(self,adr):
	    return bus.read_byte_data(address, adr)
	
	def read_word(self,adr):
	    high = bus.read_byte_data(address, adr)
	    low = bus.read_byte_data(address, adr+1)
	    val = (high << 8) + low
	    return val
	
	def read_word_2c(self,adr):
	    val = self.read_word(adr)
	    if (val >= 0x8000):
	        return -((65535 - val) + 1)#32768 to -32768
	    else:
	        return val
	
	def write_byte(self,adr, value):
	    bus.write_byte_data(address, adr, value)

	
	def get_heading(self):
		self.x_out = (self.read_word_2c(3) - self.x_offset) * self.scale  #why 0.92?
		
		self.y_out = (self.read_word_2c(7) - self.y_offset) * self.scale  #what's offset?
		
		self.z_out = (self.read_word_2c(5)) * self.scale
		
		bearing  = math.atan2(self.y_out, self.x_out) 
		if (bearing < 0):
			bearing += 2 * math.pi
		return np.rad2deg(bearing)
		#print 'bearing = ',np.rad2deg(bearing)
	def get_corr(self):
		corr=[]
		corr.append((self.read_word_2c(3) - self.x_offset)*self.scale)
		corr.append((self.read_word_2c(7) - self.y_offset)*self.scale)
		return corr
		
		

		
'''

compass = Compass()

for x in range(20):

	print 'heading', compass.get_heading()
	print 'corr is',compass.get_corr()
	time.sleep(1)
'''
