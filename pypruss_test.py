import pypruss
import time
import struct #for sturct.unpack

offset=0
class PyprussTest():
	
	def __init__(self):

		pypruss.modprobe(100)

		print 'Initializing PRU'
		pypruss.init()

		print 'successfully initialized!'

		if pypruss.open(0):
			print 'PRU open failed'
			return 1


		pypruss.pruintc_init()

		self.pruData=pypruss.map_prumem(pypruss.PRUSS0_PRU0_DATARAM)

		pypruss.exec_program(0, "./hcsr04.bin") 
	
	def get_distances(self):
		
		distances=[]

		pypruss.wait_for_event(0)                                   # Wait for event 0 which is connected to PRU0_ARM_INTERRUPT
		pypruss.clear_event(0,pypruss.PRU0_ARM_INTERRUPT)           # Clear the event
		
		time.sleep(0.2)

		for j in range(5):
			distances.append(round(float(struct.unpack('L', self.pruData[offset+j*4:offset+(j+1)*4])[0]/58.44),2))
		return distances

	def stop_pru(self):
	
		pypruss.pru_disable(0)               
		pypruss.exit()
		return 0

		
'''
pyprusstest=PyprussTest()

i=pyprusstest.get_distances()
print i
j=pyprusstest.stop_pru()
'''
