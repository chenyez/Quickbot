import time
# import pins names for ir distance sensors, 2 dc motors, and encoders.
import config
# import python math library
import numpy as np
# import adafruit's beaglebone black gpio library
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

import signal
import sys
# compass class for heading
from Compass_HMC5883L import *
from scipy.stats import mode

class QuickBot:

    def __init__(self, leftMotorPinList,rightMotorPinList, irPinList ):
        
        print 'Initializing QuickBot - Running directly on the BeagleBone'
        
         # Initialize GPIO pins
        print 'Initializing GPIO pins'
        
        self.leftMotorPinList = leftMotorPinList
        self.rightMotorPinList = rightMotorPinList
        self.irPinList = irPinList
#        self.coeffs = [-274.082,467.0223,-270.152,61.9435]  #coeffs of dis-vol function of ir
#	self.coeffs=[-549586.614410014,2408274.92579189,-4572372.37077631,4929699.72432297,-3323046.43395288,1452630.42510176,-412767.715611524,74027.2686432297,-3323046.43395288,1452630.42510176,-412767.715611524,74027.2686459216,-7743.85714142088,386.267432882943]
	self.coeffs=[-2440.04872226593,6350.17325967952,-6443.65540697229,3219.06585379877,-818.085638692615,97.2435572925929]
        self.compass = Compass();

        GPIO.setup(self.leftMotorPinList[0], GPIO.OUT)   #set pin as output
        GPIO.setup(self.leftMotorPinList[1], GPIO.OUT)
            
        GPIO.setup(self.rightMotorPinList[0], GPIO.OUT)
        GPIO.setup(self.rightMotorPinList[1], GPIO.OUT)
    
        # set the dir pins low    
        GPIO.output(self.leftMotorPinList[0], GPIO.LOW)   #set output value LOW
        GPIO.output(self.leftMotorPinList[1], GPIO.LOW)
            
        GPIO.output(self.rightMotorPinList[0], GPIO.LOW)
        GPIO.output(self.rightMotorPinList[1], GPIO.LOW)
        
        # setup ADC
        print 'Setting Up ADC'
        ADC.setup()

    """
       sets pwm for both left and right motor.
            convention: positive pwm means forward motion, negative is backwards.
    """
    def set_pwm(self, pwml, pwmr):#if pwml<0 pwmr>0, turn right, otherwise left, abs(pwml/r) decide speed
        
        # Check bounds
        if pwml > 40:
            pwml = 40
        elif pwml < -40:
            pwml = -40
        if pwmr > 40:
            pwmr = 40
        elif pwmr < -40:
            pwmr = -40
        
        print 'setting pwm = ', pwml, ', ', pwmr
        PWM.start(self.leftMotorPinList[2], 0)  #what's the information in the left/rightMotorPinList? Ans:4 switch signals for H bridge
        PWM.start(self.rightMotorPinList[2], 0) #what's the function of third Pinlish?
        # set directions
        #check if left motor is to be  negative pwm
        if pwml < 0:
            # inputs for backward motion of left motor    
            GPIO.output(self.leftMotorPinList[1], GPIO.LOW)
            GPIO.output(self.leftMotorPinList[0], GPIO.HIGH)
        else:    
            # inputs for forward motion of left motor
            GPIO.output(self.leftMotorPinList[1], GPIO.HIGH)
            GPIO.output(self.leftMotorPinList[0], GPIO.LOW)

        if pwmr < 0:
            # inputs for backward motion of right motor    
            GPIO.output(self.rightMotorPinList[1], GPIO.LOW)
            GPIO.output(self.rightMotorPinList[0], GPIO.HIGH)
        else:
            # inputs for forward motion of right motor
            GPIO.output(self.rightMotorPinList[1], GPIO.HIGH)
            GPIO.output(self.rightMotorPinList[0], GPIO.LOW)
	#make robot stop
	if pwml==0 and pwmr==0:
#	    self.stop()	
	    GPIO.output(self.rightMotorPinList[1], GPIO.LOW)
            GPIO.output(self.rightMotorPinList[0], GPIO.LOW)


        # set absolute values of pwm for both motors
        PWM.set_duty_cycle(self.leftMotorPinList[2], abs(pwml)) # left motor
        PWM.set_duty_cycle(self.rightMotorPinList[2], abs(pwmr)) # right motor

    def stop(self):
        print 'Stopping QuickBot ...'
        # set the dir pins low    
        GPIO.output(self.leftMotorPinList[0], GPIO.LOW)
        GPIO.output(self.leftMotorPinList[1], GPIO.LOW)
            
        GPIO.output(self.rightMotorPinList[0], GPIO.LOW)
        GPIO.output(self.rightMotorPinList[1], GPIO.LOW)
        
        # set o pwm to stop motors
        PWM.set_duty_cycle(self.leftMotorPinList[2], 0) # left motor
        PWM.set_duty_cycle(self.rightMotorPinList[2], 0) # right motor
        
        GPIO.cleanup()
        PWM.cleanup()
    # get IR sensor values in analog voltage
    def get_IR_volts(self):
        ir_volts = np.zeros(len(self.irPinList))
#	n = 20 
	ir_v=[[] for i in range(5)]
     
#        for x in range(n):
        for i,p in enumerate(self.irPinList):   
	    ir_volts[i]=ADC.read(p)*1.8
            ir_v[i].append(round(ir_volts[i],2))
#           time.sleep(0.01)
#	for i,p in enumerate(self.irPinList):   ##
#        	ir_volts[i] = ADC.read(p)
#        time.sleep(0.01)

#	print 'ir_v=',ir_v
	ir_volts_mode=[]
	ir_temp=[]
	for k in range(5):
		ir_temp=mode(ir_v[k])
		ir_volts_mode.append(ir_temp[0][0])
#	print 'ir_volts_mode=',ir_volts_mode
	return ir_volts_mode
#        return ir_volts*1.8/n
    # get IR sensor distances in centimeters cm
    def get_IR_distances(self):
        ir_volts = self.get_IR_volts();
#	print ir_volts
        ir_dists = np.polyval(self.coeffs, ir_volts) #each coeffs*ir_volts
#	print ir_dists
        return ir_dists
    # get robot heading x axis w.r.t north, degrees

    def get_heading(self):
        return self.compass.get_heading()
	
    def ao_heading(self):
	robot_heading=self.get_heading()
#	print 'robot heading=', robot_heading
	ir_heading=[robot_heading-90,robot_heading-45,robot_heading,robot_heading+45,robot_heading+90]
#	print 'ir_headings=', ir_heading
	ir_dist=self.get_IR_distances()
	print 'ir_dist=', ir_dist
	ir_corr=[]
	for i in range(5):
		ir_corr.append(self.compass.ir_to_world(ir_dist[i],ir_heading[i]))
#	print 'ir_corr=', ir_corr
	#	ir_corr[i].append(ir_dist[i]*math.sin(ir_heading[i]))
	#	ir_corr[i].append(ir_dist[i]*math.cos(ir_heading[i]))
	#	ao_dir=map(lambda(a,b):a+b, zip(ao_dir,ir_corr[i]))
#	print 'ir_correlates = ', ir_corr
#	print 'objective direction = ',ao_dir
#	ao_headings=-180*math.atan2(ao_dir[1],ao_dir[0])/math.pi
#	print 'avoid obstacle heading =',ao_headings
#	return ao_heading

				
#	print 'ir_dist=', ir_dist
#store two ir sensors which detects the obstacles
	ob_detect=[]
	
	ob_angle=0.0
	for i in range(5):
		if ir_dist[i]<9:
			ob_detect.append(i)
	print 'ob_detect', ob_detect
	
#	u_a=[]	#correlate of first intersection of obstacle and ir_sensors
#	u_b=[]  #correlate of second intersection of obstacle and ir_sensors
	u_p=[]
	wf=[]
	wf_t=[]
	wf_t_u=[]
	wf_p=[]
	wf_p_u=[]
	wf_u=[]
	
	u_p=self.compass.ir_to_world(0,ir_heading[2])#correlate of the robot in world frame
#	print 'ir_corr', ir_corr
	if len(ob_detect)!=0:
		if len(ob_detect)>1:
	#		ob_angle=ir_heading[ob_detect[0]]-90 #turn right
	#		print 'ob_detect=, ir_heading=', ob_detect[0], ir_heading[ob_detect[0]]
	#	else:
		#	u_a=self.compass.ir_to_world(ir_dist[ob_detect[0]],ir_heading[ob_detect[0]])
		#	print 'u_a=', u_a
		#	u_b=self.compass.ir_to_world(ir_dist[ob_detect[1]],ir_heading[ob_detect[1]])
		#	print 'u_b=', u_b
			wf_t=np.subtract(ir_corr[ob_detect[0]],ir_corr[ob_detect[1]])
		#	print 'wf_t=',wf_t
			wf_t_u=wf_t/np.linalg.norm(wf_t)
		#	print 'wf_t_u=',wf_t_u

		#	ob_angle=180*math.atan2(ob_dir[1],ob_dir[0])/math.pi
			wf_p_temp=np.subtract(ir_corr[ob_detect[0]],u_p)
			wf_p=np.subtract(wf_p_temp,np.dot(np.dot(wf_p_temp,wf_t_u),wf_t_u))
		#	print 'wf_p=', wf_p
			wf_p_u=wf_p/np.linalg.norm(wf_p)
		#	print 'wf_p_u=',wf_p_u

			wf=0.05*wf_p_u+0.95*wf_t_u

			wf_u=wf/np.linalg.norm(wf)
		#	print 'wf_u=', wf_u		
		
	return wf_u
'''
quik=QuickBot(config.LEFT_MOTOR_PINS,config.RIGHT_MOTOR_PINS, config.IR_PINS)
#ir_d=quik.get_IR_distances()
#ir_sum=0
ir_2=[]
#for i in range(10):
#ir_2=quik.get_IR_volts()
ir_2=quik.get_IR_distances()
print 'ir_2=',ir_2
'''
'''
for x in range(200):
	ir_d = quik.get_IR_volts()
	
	ir_2.append(round(ir_d[2],2))

	print 'ir_distances=',ir_d
	print 'ir_dis=', ir_2[x]
	ir_sum+=ir_2[x]

ir_avg=ir_sum/200

print 'ir_avg=',ir_avg

print 'ir_mode=',mode(ir_2)
'''
