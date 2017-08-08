import Adafruit_BBIO.PWM as PWM
import time

servor='P9_16'
servol='P8_19'


class Servo():

	def __init__(self):
	
		PWM.start(servor, 100, 50)
		PWM.start(servol, 100, 50)

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
'''
servo=Servo()
servo.set_pwm(10,10)
time.sleep(3)
servo.set_pwm(70,50)
time.sleep(3)
servo.set_pwm(100,100)
time.sleep(3)
servo.set_pwm(0,0)
'''
