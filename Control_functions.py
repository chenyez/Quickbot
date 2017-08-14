import time

import math
# import python math library
import numpy as np
# import adafruit's beaglebone black gpio library

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

import signal
import sys
# compass class for heading
from Compass_HMC5883L import Compass
#import Compass_HMC5883L
#from scipy.stats import mode
from pypruss_test import *
from servoctl import *



class ControlFunctions:

    def __init__(self):
        
        print 'Initializing QuickBot - Running directly on the BeagleBone'
        
        self.hc_dist=[]
        self.compass = Compass()
	self.servo = Servo()
	self.pypruss =  PyprussTest()
	


    def set_speed(self,pwml,pwmr):
	self.servo.set_pwm(pwml,pwmr)

    def stop(self):
        servo.ser_pwm(0,0)

    def get_heading(self):
        return self.compass.get_heading()

    def get_distances(self):
	return self.pypruss.get_distances()

    def dotproduct(self, v1, v2):
	return sum((a*b) for a, b in zip(v1, v2))

    def length(self, v):
	return math.sqrt(self.dotproduct(v, v))

    def angle(self,v1, v2):
	return np.rad2deg(math.acos(self.dotproduct(v1, v2) / (self.length(v1) * self.length(v2))))

	
    def ao_heading(self):
	
	
	robot_heading=self.get_heading()

 



	print 'robot_headings is:',robot_heading
	
	

	self.hc_dist=self.get_distances()
	print 'hc_dists=',self.hc_dist



				
	ob_detect=[]
	
	ob_angle=0.0
	for i in range(5):
		if self.hc_dist[i]<20:
			ob_detect.append(i)
	print 'ob_detect', ob_detect

	angle_turn=1

	if len(ob_detect)!=0:
		if len(ob_detect)==1:
			print 'only one'
		else:


			ob1_x=np.multiply(self.hc_dist[ob_detect[0]],math.cos(ob_detect[0]*math.pi/4))
			ob1_y=np.multiply(self.hc_dist[ob_detect[0]],math.sin(ob_detect[0]*math.pi/4))

			ob2_x=np.multiply(self.hc_dist[ob_detect[1]],math.cos(ob_detect[1]*math.pi/4))
			ob2_y=np.multiply(self.hc_dist[ob_detect[1]],math.sin(ob_detect[1]*math.pi/4))

			ob1_corr=[ob1_x,ob1_y]
			ob2_corr=[ob2_x,ob2_y]

			ob_heading=np.subtract(ob2_corr,ob1_corr)

			print 'ob_heading is: ',ob_heading
			rb_heading=[0,1]
		
			angle_turn=self.angle(ob_heading,rb_heading)



	return angle_turn





if __name__ == "__main__":
	controlfunctions=ControlFunctions()
	angle=controlfunctions.ao_heading()
	print 'angle turn is :',angle
	
	controlfunctions.pypruss.stop_pru()
