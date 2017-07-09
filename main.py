from qb_zcycode import *
import time

nav=Navigation()

try:
	while nav.navState != NavState.HALT:

	#	ao_heading=nav.wf_heading()
	#	print 'ob_angle=', ao_heading
		nav.run()
#		time.sleep(0.001)
		
except KeyboardInterrupt:
	print 'keyboard interrupt received.. stopping the quickbot'
	nav.stop()

