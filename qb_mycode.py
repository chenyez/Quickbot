"""
"""

from QuickBot import *
		
from enum import Enum

import time
import numpy as np

class NavState(Enum):
	 GO_TO_GOAL = 0
	 PRE_WALL_FOLLOW = 1
	 WALL_FOLLOW = 2
	 HALT = 3
	 		
class Navigation:
	

	def __init__(self):
		 
		self.qb = QuickBot(config.LEFT_MOTOR_PINS, config.RIGHT_MOTOR_PINS, config.IR_PINS)
		self.navState = NavState.GO_TO_GOAL
		self.headingBeforeWall = 0
		self.targetHeadingForWallFollow = 0
		self.currentHeading = 0
		self.proximities = [0.0,0.0,0.0,0.0,0.0]
		self.goalHeading = 320
		self.wallFollowMin = 15
		self.wallFollowMax = 20
	# updates the physical state of the robot: heading and proximities
	def update(self):
		self.currentHeading = self.qb.get_heading()
		self.proximities     = self.qb.get_IR_distances()
	def printState(self):
		print 'Proximities = ', self.proximities, 'Heading = ', self.currentHeading, 'Degrees'
	def goToGoal(self, angle):
		
			
		currentHeading = self.qb.get_heading()
		
		print 'current heading = ',currentHeading
			
		err = currentHeading - angle;

		print 'err is = ', err
		
		if err > 180 :
			err = err - 360
		elif err < -180 : 
			err = err + 360
			
		kp = 0.5
		
		self.qb.set_pwm(np.clip(50+kp*err,-60,60),np.clip(50-kp*err,-60,60));	

	def run(self):
		ir_dists = self.proximities;
		print 'Current State is ', self.navState.name	
		if self.navState == NavState.GO_TO_GOAL:
			if ir_dists[2] < 15 :
				self.headingBeforeWall = self.qb.get_heading();
				self.targetHeadingForWallFollow = self.headingBeforeWall - 80
				if self.targetHeadingForWallFollow < 0:
					self.targetHeadingForWallFollow += 360
				self.navState = NavState.PRE_WALL_FOLLOW
				self.qb.set_pwm(0,0)
				return

			self.goToGoal(self.goalHeading)
		elif self.navState == NavState.PRE_WALL_FOLLOW:
			currentHeading = self.qb.get_heading();
			eps = 20	
			if currentHeading > self.targetHeadingForWallFollow-eps and currentHeading < self.targetHeadingForWallFollow+eps:
				self.qb.set_pwm(0,0)
				self.navState = NavState.WALL_FOLLOW
				#turn right
				return
			self.qb.set_pwm(60,-60)
 
		elif self.navState == NavState.WALL_FOLLOW:
			print 'WALL FOLLOW MODE, distances = ', ir_dists
			ir_dists_wf = [self.validate(x,30) for x in ir_dists ]	
			#wfx = ir_dists_wf[1]*0.707 + ir_dists_wf[2]+ir_dists_wf[3]*0.707	
			#wfy = ir_dists_wf[0] + ir_dists_wf[1]*0.707 - ir_dists_wf[3]*0.707 - ir_dists_wf[4]
			
		#	wfx = 
		#	wf_angle = -np.rad2deg( np.arctan2(wfy,wfx) )
			
		#	print 'WALL FOLsLOW ANGLE = ', wf_angle, 'degrees'
			if ir_dists[0] > self.wallFollowMax:
				#turn left
				self.qb.set_pwm(40,50)
			elif ir_dists[0] < self.wallFollowMin :
				# turn right
				self.qb.set_pwm(50,40)
			else:
				self.qb.set_pwm(45,45)
			
		#	self.qb.set_pwm(40+np.clip(2*wf_angle,-70,70),40-np.clip(-2*wf_angle,-70,70));
 	def stop(self):
 		
		self.qb.stop()
	
	# if x > max, x = 0
	def validate(self,x,maxi):
		if x > maxi:
			return 0
		return x



	
# main code

nav = Navigation()

try:
	while nav.navState != NavState.HALT:
		nav.update()
		nav.printState()	
		nav.run()
		time.sleep(0.01)
		
except KeyboardInterrupt:
	print 'keyboard interrupt received.. stopping the quickbot'
	nav.stop()
	

	
	

#if not obs:
#	qb.stop()
	
		
		
			


