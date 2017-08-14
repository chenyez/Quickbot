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

if __name__ == "__main__":

	servo=Servo()
	print 'Test1'

	servo.set_pwm(1,60)
	time.sleep(2)

	print 'Test2'
	servo.set_pwm(0,0)
	time.sleep(2)

	servo.set_pwm(1,60)
	time.sleep(2)

	servo.set_pwm(0,0)
	time.sleep(2)

	print 'Test3'

	servo.set_pwm(1,60)
	time.sleep(2)

	servo.set_pwm(0,0)
	time.sleep(1)

	print 'Test4'

	servo.set_pwm(1,60)
	time.sleep(2)
	print 'change speed'

	servo.set_pwm(0,0)

'''
left: 15>10

right: 28<30

left&right: 10>28, 15>30, 10>29,70<50,70<35
'''
