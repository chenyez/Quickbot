	print 'Current state is: ',self.navState
				wf_turn = self.control_func.ao_heading(hc_dist,ob_detect)
				print "wf_turn is :", wf_turn
				
				time.sleep(1)
				
				self.control_func.servo.angle_turns(wf_turn)

				self.navState='GOAL_TO_GOAL'
				print 'current state is:', self.navState

				
				wf_heading=self.control_func.compass.get_heading()
				print wf_heading
				
				self.control_func.servo.set_pwm(0.1,30)
			#	self.go_to_goal(wf_heading)

