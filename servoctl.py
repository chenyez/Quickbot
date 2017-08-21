import Adafruit_BBIO.PWM as PWM
import time
from Compass_HMC5883L import *
servor='P9_16'
servol='P8_19'


class Servo():

	def __init__(self):
	
		PWM.start(servor, 0,500)
		PWM.start(servol, 0,500)
		self.compass=Compass()

	def set_pwm(self, pwml, pwmr):



		if pwml>100:
			pwml=100
		elif pwml<-100:
			pwml=-100

		if pwmr>100:
			pwmr=100
		elif pwmr<-100:
			pwmr=-100



		PWM.set_duty_cycle(servor,100-abs(pwmr))
		PWM.set_duty_cycle(servol,100-abs(pwml))


#		PWM.set_duty_cycle(servor,pwmr)
#		PWM.set_duty_cycle(servol,pwml)


	def angle_turns(self,angle):


		num_ninety=angle/90.0
		self.set_pwm(30,0)
		time.sleep(0.6*num_ninety)
		print 'num_ninety= ',num_ninety

		self.set_pwm(0,0)
		print 'end turning'


	def angle_turns_right(self,angle):


		num_ninety=angle/90.0
		self.set_pwm(0,0.1)
		time.sleep(0.6*num_ninety)
		print 'num_ninety= ',num_ninety

		self.set_pwm(0,0)
		print 'end turning'

	
'''

		if angle>90:
			num_ninety=angle/90.0
			self.set_pwm(0,0.1)
			time.sleep(0.6*num_ninety)
			print 'num_ninety= ',num_ninety

			self.set_pwm(0,0)
			print 'end turning'
		else:
			num_ninety=(180-angle)/90.0
			self.set_pwm(30,0)
			time.sleep(0.65*num_ninety)
			print 'num_ninety= ',num_ninety
			self.set_pwm(0,0)
			print 'end turning'
	#	a='turn set'
		return 

'''		

if __name__ == "__main__":



	servo=Servo()
	print 'current heading: ',servo.compass.get_heading()
	

	servo.set_pwm(0,0.1)
	time.sleep(0.65)
	

	servo.set_pwm(0,0)
	
	print 'after turn:',servo.compass.get_heading()
'''
	print 'Test2'
	servo.set_pwm(0,0)
	time.sleep(2)

	servo.set_pwm(0.1,30)
	time.sleep(2)

	servo.set_pwm(0,0)
	time.sleep(2)

	print 'Test3'

	servo.set_pwm(0.1,30)
	time.sleep(2)

	servo.set_pwm(0,0)
	time.sleep(1)


	servo.set_pwm(0,0)

'''
'''
left: 15>10

left 90 degree: set_pwm(0,30) 0.6sec
right 90 degree: set_pwm(0.1,0) for 0.6 sec 


right: 28<30

left&right: 10>28, 15>30, 10>29,70<50,70<35
'''
