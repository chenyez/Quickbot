"""
Mike Kroutikov, (c) 2014

Test QuickBot IR sensors.

Run this code on the BeagleBone side.

It will display for about 30 seconds the current values of all five sensors.

While this program runs, try putting an obstacle before each sensor. Unobstructed sensor should read
low value (e.g. 0). Placing palm of your hand at about 2 inches should cause large readings (300-700).

"""
import Adafruit_BBIO.ADC as ADC
import time
import sys
import numpy as np
onek = 10**3
coeffs = [-274.082,467.0223,-270.152,61.9435]
if __name__ == '__main__':
    import config

    print 'Num of args = ',len(sys.argv)
    print 'Arg list = ',str(sys.argv)
    print "Testing IR sensors. Try putting hand in front of a sensor to see its value change"

    def polyval(coeff,x):
    	res=0
    	deg = len(coeff)-1
    	for c in coeff:
    		res = res+c*(x**deg)
    		deg = deg-1
	return res
	
	
    ADC.setup()
    num = 20
    values_v = (0.0,0.0,0.0,0.0,0.0)
    time_bef = time.time()
    for _ in range(num):
        values = tuple(ADC.read_raw(pin) for pin in config.IR_PINS)
        v = tuple(ADC.read(pin) for pin in config.IR_PINS)
	values_v = tuple(v[i]+values_v[i] for i in range(0,len(v)) ) 
	time.sleep(0.05)
        #print 'Raw sensor values: %4d %4d %4d %4d %4d' % values
        #print 'Raw sensor values: %4f %4f %4f %4f %4f' % values

    values_v = tuple(1.8*val/num for val in values_v)
	
    #dists = tuple(polyval(coeffs,vals) for vals in values_v)	
    dists = tuple(np.polyval(coeffs,val) for val in values_v)
    print 'Average sensor Analog values volts: %4f %4f %4f %4f %4f' % values_v
    print 'Actual distances: %4f %4f %4f %4f %4f' % dists
    print "Done in ",time.time()-time_bef," secs"
