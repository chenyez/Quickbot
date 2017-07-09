from QuickBot import *
import numpy as np
import math

class IR_heading:
	
	def __init__(self):
		self.qbb = QuickBot(config.LEFT_MOTOR_PINS,config.RIGHT_MOTOR_PINS, config.IR_PINS)
		
		self.robot_heading=0
		self.ir_heading=[0.0,0.0,0.0,0.0,0.0]
		self.ir_dist=[0.0,0.0,0.0,0.0,0.0]
	
	def angle_turn(self):
		robot_heading=self.qbb.get_heading()
	#	print 'robot heading=', robot_heading
		ir_heading=[robot_heading-90,robot_heading-45,robot_heading,robot_heading+45,robot_heading+90]
	#	print 'ir_headings=', ir_heading
		ir_dist=self.qbb.get_IR_distances()
	#	print 'ir_dist=', ir_dist

		ir_corr=[[] for i in range(5)]
		ao_dir=[0.0,0.0]
		for i in range(5):
			ir_corr[i].append(ir_dist[i]*math.sin(ir_heading[i]))
			ir_corr[i].append(ir_dist[i]*math.cos(ir_heading[i]))
			ao_dir=map(lambda(a,b):a+b, zip(ao_dir,ir_corr[i]))
	#	print 'ir_correlates = ', ir_corr
	#	print 'objective direction = ',ao_dir
	#	ao_headings=-180*math.atan2(ao_dir[1],ao_dir[0])/math.pi
	#	print 'avoid obstacle heading =',ao_headings
	#	return ao_heading

				
	#	print 'ir_dist=', ir_dist
	#store two ir sensors which detects the obstacles
		ob_detect=[]
		ob_dir=[0.0,0.0]
		ob_angle=0.0
		for i in range(5):
			if ir_dist[i]<9:
				ob_detect.append(i)
		print 'ob_detect', ob_detect
		
		
	#	print 'ir_corr', ir_corr
		if len(ob_detect)!=0:
			if len(ob_detect)==1:
				ob_angle=ir_heading[ob_detect[0]]-90 #turn right
				print 'ob_detect=', ob_detect[0], 'ir_heading=', ir_heading[ob_detect[0]]
			else:
				ob_dir=np.subtract(ir_corr[ob_detect[0]],ir_corr[ob_detect[1]])
				ob_angle=math.atan2(ob_dir[1],ob_dir[0])
			if ob_angle<0:
				ob_angle += 2 * math.pi
			ob_angle=np.rad2deg(ob_angle)
		print 'ob_dir', ob_dir,'ob_angl=e', ob_angle
		return ob_angle

		


#main code
irh= IR_heading()
ob_angle=0
try:
	while 1:
  	#	irh.ao_heading()
		ob_angle=irh.angle_turn()
	#	print 'ob_angle=', ob_angle
		time.sleep(0.01)
except KeyboardInterrupt:
	print 'KeyBoard Interruption'
	irh.qbb.stop() 

