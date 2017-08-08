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

	def get_corr(self):
		corr=[]
		corr.append((self.read_word_2c(3) - self.x_offset)*self.scale)
		corr.append((self.read_word_2c(7) - self.y_offset)*self.scale)
		return corr

	def hc_corr(self):
		[x,y]=self.get_corr()
		hc_corr=[[] for i in range(5)]
		hc_corr[0]=[x-25.76,y-121.44]
		hc_corr[1]=[x-132.48,y+20.24]
		hc_corr[2]=[x-216.2,y+44.16]
		hc_corr[3]=[x+359.72,y+119.6]
		hc_corr[4]=[x+278.76,y-152.72]
		return hc_corr
	
	def get_angle(self,x,y):
		theta=math.atan2(y,x)
		if (theta < 0):
			theta += 2 * math.pi
		return np.rad2deg(theta)

	def get_heading(self):
		hc_cor=self.hc_corr()
		get_cor=self.get_corr()
		print 'hc_cor[0]=', hc_cor[0], 'get_cor=',get_cor
		
		comp_to_zero=np.subtract(hc_cor[0],get_cor)
		heading_robot=self.get_angle(comp_to_zero[0],comp_to_zero[1])
		heading_sonars=[heading_robot-90,heading_robot-45,heading_robot,heading_robot+45,heading_robot+90]
		return heading_sonars

'''
	def hc_to_world(self, di, theta_hc):
		x= (self.read_word_2c(3) - self.x_offset) * self.scale 
		y= (self.read_word_2c(7) - self.y_offset) * self.scale
		theta_rb=self.get_heading()

		R=self.frame_transmission_matrix(x,y,theta_rb)
		R_si=self.frame_transmission_matrix(0,0,theta_rb)

		hc_sensor=np.array([[di],[0],[1]])
	#	print 'ir_sensor=',ir_sensor
		hc_robot=np.dot(R_si,hc_sensor)
		hc_world=np.dot(R,hc_robot)
	#	print 'ir_world=', ir_world
		hc_corr=np.transpose(np.array([hc_world[0],hc_world[1]]))	
		return hc_corr[0]
		
		

	def frame_transmission_matrix(self,x,y,theta):	
		theta_d=math.radians(theta)
		R=np.array([[math.cos(theta_d),-math.sin(theta_d),x],[math.sin(theta_d),math.cos(theta_d),y],[0,0,1]])
		return R
'''		


compass = Compass()

x_out = (compass.read_word_2c(3) - compass.x_offset) * compass.scale  #why 0.92?
		
y_out = (compass.read_word_2c(7) - compass.y_offset) * compass.scale  #what's offset?
print 'x_out=',x_out
print 'y_out=',y_out

corr=compass.get_heading()
print corr
'''
heading=compass.get_heading()
print 'heading=',heading
		
#matrix=compass.frame_transmission_matrix(1,2,60)
#print 'matrix=',matrix
#print 'heading', compass.get_heading()
print 'x_out=',x_out
print 'y_out=',y_out
ir_w=[]
#ir_w_temp=[]
ir_w=compass.hc_to_world(1,90)
#ir_w[0].append(ir_w_temp[0])
#ir_w[0].append(ir_w_temp[1])

print 'ir_w=', ir_w
'''
