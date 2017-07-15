import Adafruit_BBIO.PWM as PWM
import time

class Servol:

	servolr='P9_16'
	servoll='P9_14'
	PWM.start(servolr, 100, 50)
	PWM.start(servoll, 100, 50)

	def set_pwm(self, pwml, pwmr):

		if pwml>100:
			pwml=100
		elif pwml<-100:
			pwml=-100

		if pwmr>100:
			pwmr=100
		elif pwmr<-100:
			pwmr=-100



		PWM.set_duty_cycle(self.servolr,100-abs(pwmr))
		PWM.set_duty_cycle(self.servoll,100-abs(pwml))

servol=Servol()
servol.set_pwm(100,0)
time.sleep(3)
servol.set_pwm(0,100)
time.sleep(3)
servol.set_pwm(100,100)
time.sleep(3)
servol.set_pwm(0,0)
