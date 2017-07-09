from QuickBot import *
from enum import Enum

class NavState(Enum):
	GO_TO_GOAL=0
	PRE_WALL_FOLLOW=1
	WALL_FOLLOW=2
	HALT=3
#	AVOID_OBSTACLES=4

class Navigation:

	def __init__(self):
		self.q_b = QuickBot(config.LEFT_MOTOR_PINS,config.RIGHT_MOTOR_PINS, config.IR_PINS)
	#	self.qb = QuickBot(config.LEFT_MOTOR_PINS,config.RIGHT_MOTOR_PINS, config.IR_PINS)
		self.navState='GO_TO_GOAL'
		self.currentHeading=0
		self.Headingerror=0
		self.goalHeading=90
		self.ir_dist=[0.0,0.0,0.0,0.0,0.0]

	def go_to_goal(self,angle):
	#	print 'gtg function start'
		currentHeading=self.q_b.get_heading()
		print 'currentHeading is', currentHeading

		Headingerror=currentHeading-angle
		print 'error is', Headingerror

		if Headingerror>180:
			Headingerror=Headingerror-360	
		elif Headingerror<-180:
			Headingerror=Headingerror+360	

		error_P=Headingerror
		kp=0.3
	#	print 'pwm l=',60+ kp*error_P, ' pwm r=', 60-kp*error_P
		self.q_b.set_pwm(np.clip(30+kp*error_P,-60,60),np.clip(30-kp*error_P,60,60))
	#	self.q_b.set_pwm(40+kp*error_P,40-kp*error_P)
	#	print 'gtg function end'
		print 'ir_dists=', self.q_b.get_IR_distances()
	
	def wf_heading(self):
		return self.q_b.ao_heading()
	def stop(self):
		self.q_b.stop()
	def run(self):
		ir_dist=self.q_b.get_IR_distances()
		currentHeading=self.q_b.get_heading()
		if np.amin(self.q_b.get_heading())<10:
			print 'ir_dist=',self.q_b.get_heading()
			self.q_b.stop()

		
		if self.navState=='GO_TO_GOAL':
	#		self.q_b.set_pwm(0,0)

			print 'seconde ir_dists=', self.q_b.get_IR_distances()
			ir_dis=self.q_b.get_IR_distances()
			if ir_dis[1]<10 or ir_dis[2]<10 or ir_dis[3]<10:
				print 'final ir_dists=', self.q_b.get_IR_distances()
				
				self.q_b.stop

				print 'Current state is: ',self.navState
				self.navState='PRE_WALL_FOLLOW'
			else:
				self.q_b.set_pwm(0,0)
				self.go_to_goal(self.goalHeading)

			return
		elif self.navState=='PRE_WALL_FOLLOW':
			self.q_b.set_pwm(0,0)
			
			print 'compare ir_dists=', self.q_b.get_IR_distances()
			print 'current heading=', self.q_b.get_heading()

		#	if len(wf_u)!=0:	
			print 'Current state is: ',self.navState			
			self.navState='PRE_WALL_FOLLOW'
			
			return
		elif self.navState=='WALL_FOLLOW':
			self.q_b.set_pwm(0,0)

			#self.go_to_goal(ob_angle)
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

		
	
	
