import Adafruit_BBIO.PWM as PWM
import time

servor='P9_16'
servol='P8_19'


class Servo():

	def __init__(self):
	
		PWM.start(servor, 0,500)
		PWM.start(servol, 0,500)

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
		print 'begin turning'
		if angle>90:
			num_ninety=(180-angle)/90.0
			self.set_pwm(0,30)
			time.sleep(0.6*num_ninety)
			self.set_pwm(0,0)
			print 'end turning'
		else:
			num_ninety=angle/90.0
			self.set_pwm(0.1,0)
			time.sleep(0.6*num_ninety)
			self.set_pwm(0,0)
			print 'end turning'
		return

		

if __name__ == "__main__":



	servo=Servo()
	print 'Test1'

	servo.set_pwm(10,30)
	time.sleep(2)

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
left: 15>10

left 90 degree: set_pwm(0,30) 0.6sec
right 90 degree: set_pwm(0.1,0) for 0.6 sec 


right: 28<30

left&right: 10>28, 15>30, 10>29,70<50,70<35
'''
