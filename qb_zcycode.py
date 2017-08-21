from Control_functions import *
from servoctl import *
#from enum import Enum
from Compass_HMC5883L import Compass
#from pypurss_test import *

'''
class NavState(Enum):
	GO_TO_GOAL=0
	PRE_WALL_FOLLOW=1
	WALL_FOLLOW=2
	HALT=3
#	AVOID_OBSTACLES=4
'''
class Navigation:

	def __init__(self):
		self.control_func= ControlFunctions()
		self.navState='GO_TO_GOAL'
		self.currentHeading=0
		self.Headingerror=0

		self.goalHeading=270

		self.hc_dist=[]
	#	self.servo=Servo()
	#	self.compass=Compass()
	#	self.pypruss =  self.control_func.pypruss
		self.wf_heading=0


	def go_to_goal(self,angle):
	#	print 'gtg function start'
		currentHeading=self.control_func.compass.get_heading()
		print 'currentHeading is', currentHeading

		Headingerror=currentHeading-angle
		print 'error is', Headingerror

		if Headingerror>180:
			Headingerror=Headingerror-360	
		elif Headingerror<-180:
			Headingerror=Headingerror+360	

		error_P=Headingerror
		kp=0.3
#		self.control_func.servo.set_pwm(np.clip(30+kp*error_P,-100,100),np.clip(30-kp*error_P,100,100))
		self.control_func.servo.set_pwm(0.1,30)

	#	self.q_b.set_pwm(40+kp*error_P,40-kp*error_P)
	#	print 'hc_dists=', self.pypruss.get_distances()
	



	def obstacle_detect(self,distances):
		ob_detect=[]
		for i in range(5):
			if distances[i]<30:
				ob_detect.append(i)
		return ob_detect
		
	def stop(self):
		self.control_func.servo.set_pwm(0,0)
		nav.control_func.pypruss.stop_pru()
		
	



	def run(self):


		currentHeading=self.control_func.compass.get_heading()
		
		if self.navState=='GO_TO_GOAL':


			hc_dist=self.control_func.pypruss.get_distances()
			print 'hc_dists=',hc_dist

			ob_detect=self.obstacle_detect(hc_dist)
			
			if len(ob_detect)>1:
				
				self.control_func.servo.set_pwm(0,0)
				print 'Current state is: ',self.navState
				wf_turn = self.control_func.ao_heading(hc_dist,ob_detect)
				print "wf_turn is :", wf_turn
				
				time.sleep(1)
				
				self.control_func.servo.angle_turns(wf_turn)	
			


				
				self.navState='WALL_FOLLOW'
				print 'current state is:', self.navState
				return
				
			#	wf_heading=self.control_func.compass.get_heading()
			#	print wf_heading
				
			#	self.control_func.servo.set_pwm(0.1,30)
			#	self.go_to_goal(wf_heading)
	
			elif len(ob_detect)==1:
			#	self.control_func.servo.set_pwm(0.1,30)
				time.sleep(0.8)
				self.control_func.servo.set_pwm(0,0)
				
				i=ob_detect[0]
				if i==4:
					self.control_func.servo.set_pwm(0.1,30)
				else:	
					self.control_func.servo.angle_turns((4-i)*5)
					
			#	self.control_func.servo.set_pwm(0.1,30)
			#	time.sleep(0.1)
				return
				
				
				
	
			else:
			
				self.control_func.servo.set_pwm(0.1,30)
				self.go_to_goal(self.goalHeading)
				self.navState='GO_TO_GOAL'
				return

		elif self.navState=='PRE_WALL_FOLLOW':
			self.stop()	
			print 'stop'
		
			return


		elif self.navState=='WALL_FOLLOW':

			self.control_func.servo.set_pwm(0,0)
			
			hc_dist=self.control_func.pypruss.get_distances()
			ob_detect=self.obstacle_detect(hc_dist)
			print 'ob_detect in WALL_FOLLOW', ob_detect
			
			if len(ob_detect)>1:
				wf_turn = self.control_func.ao_heading(hc_dist,ob_detect)				
				time.sleep(1)				
				self.control_func.servo.angle_turns(wf_turn)
								
				self.control_func.servo.set_pwm(0.1,30)
				time.sleep(0.1)
				
				self.navState='WALL_FOLLOW'
			elif len(ob_detect)==1:
				if hc_dist[4]>35:
					time.sleep(1)				
					self.control_func.servo.angle_turns_right(10)
	
				self.control_func.servo.set_pwm(0.1,30)
				time.sleep(0.1)
				self.navState='WALL_FOLLOW'

			else:
				self.navState='GO_TO_GOAL'
			return			




nav=Navigation()

try:
	while 1:
	
		nav.run()
		time.sleep(0.01)
		
except KeyboardInterrupt:
	print 'keyboard interrupt received.. stopping the quickbot'
	nav.stop()

	
	
	
