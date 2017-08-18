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
		self.servo=Servo()
		self.compass=Compass()
		self.pypruss =  self.control_func.pypruss
		self.wf_heading=0


	def go_to_goal(self,angle):
	#	print 'gtg function start'
		currentHeading=self.compass.get_heading()
		print 'currentHeading is', currentHeading

		Headingerror=currentHeading-angle
		print 'error is', Headingerror

		if Headingerror>180:
			Headingerror=Headingerror-360	
		elif Headingerror<-180:
			Headingerror=Headingerror+360	

		error_P=Headingerror
		kp=0.3
#		self.servo.set_pwm(np.clip(30+kp*error_P,-100,100),np.clip(30-kp*error_P,100,100))
		self.servo.set_pwm(0.1,30)

	#	self.q_b.set_pwm(40+kp*error_P,40-kp*error_P)
		print 'hc_dists=', self.pypruss.get_distances()
	
	def wf_heading(self):
		return self.q_b.ao_heading()
	def stop(self):
		self.q_b.stop()
	def run(self):
#		hc_dist=self.pypruss.get_distances()

		currentHeading=self.compass.get_heading()
		
		if self.navState=='GO_TO_GOAL':


			hc_dist=self.pypruss.get_distances()
			print 'hc_dists=',hc_dist
#			hc_dis=[hc_dist[0],hc_dist[1],hc_dist[3],hc_dist[4]]

			ob_detect=[]
			for i in range(5):
				if hc_dist[i]<30:
					ob_detect.append(i)

#			if np.amin(hc_dist)<20: #or hc_dist[2]<7:
#				print 'hc_ditss=', hc_dist

			if len(ob_detect)>1:
				
				self.servo.set_pwm(0,0)
				
				print 'Current state is: ',self.navState
				wf_turn = self.control_func.ao_heading(hc_dist,ob_detect)
				print "wf_turn is :", wf_turn
				
				time.sleep(1)
				
				self.servo.angle_turns(wf_turn)
#				self.navState='WALL_FOLLOW'
				self.navState='GOAL_TO_GOAL'
				print 'current state is:', self.navState
				
			else:
			#	time.sleep(0.2)
				self.servo.set_pwm(0.1,30)
				self.go_to_goal(self.goalHeading)
				self.navState='GO_TO_GOAL'


			return

		elif self.navState=='PRE_WALL_FOLLOW':
			self.servo.set_pwm(0,0)

			wf_heading=self.control_func.ao_heading(hc_dist)
			print 'wall follow direction: ',wf_heading

			if wf_heading==1:
				self.servo.set_pwm(0.1,30)
				self.navState='GO_TO_GOAL'
				return

			


			print 'current heading=', self.compass.get_heading(),'wf_heading=',self.wf_heading

		#	if len(wf_u)!=0:	
			print 'Current state is: ',self.navState			
			self.navState='PRE_WALL_FOLLOW'
	

		
			return


		elif self.navState=='WALL_FOLLOW':
			self.servo.set_pwm(0,0)
			self.wf_heading=self.control_func.ao_heading()


			print 'compare hc_dists=', self.pypruss.get_distances()
			print 'current heading=', self.compass.get_heading(),'wf_heading=',self.wf_heading

		#	if len(wf_u)!=0:	
			print 'Current state is: ',self.navState	


			self.q_b.set_pwm(0,0)

			self.go_to_goal(self.wf_heading)
			self.navState='WALL_FOLLOW'
			return			
'''
			wf_u=[]
			ob_angle=0.0
			#print 'Current state is: ',self.navState
			wf_u=self.wf_heading()
			print 'wf_u=', wf_u
			if len(wf_u)!=0:
				ob_angle=math.atan2(wf_u[1],wf_u[0])
				if ob_angle<0:
					ob_angle += 2 * math.pi
				ob_angle=np.rad2deg(ob_angle)
			print 'ob_angle',  ob_angle
			print 'currentHeading=',currentHeading

			
			print 'Current state is: ',self.navState
			return
			print 'ir_dist=',self.q_b.get_IR_distances()
			print 'wf_u=', wf_u
			self.q_b.set_pwm(0,0)
'''
navi=Navigation()
i=0
while i<20:

	navi.run()
	i=i+1

navi.control_func.pypruss.stop_pru()
	
	
	
