import Serverconnect # Connect to Sensor Device
import sys
import traceback
from datetime import datetime


if __name__ == "__main__":
	try:
		
		# choose file to save to
		logname = "dataLog_" + ".txt" #str(datetime.now()) + 
		log = open(logname, "w")
		
		# choose connection type (UDP?)
		UDP_enable = False #Serverconnect.chooseType()
		
		# communikation with mikrocontroller
		print("starting Communication")
		client, s = Serverconnect.ServerSetup(port=5555, UDP=UDP_enable)
		
		print("starting Loop")
		firstReading = True
		result = {'Accelerometer': [], 'Gyroscope': [], 'Calculated Angle': []}
		running = True
		while running:
			# recieve Data
			if UDP_enable:
				data = Serverconnect.gettingUDPtext(s)
			else:
				data = Serverconnect.gettingtext(client)
			
			# stop condition
			print("data: ", data)
			if data == "" or data == "done"or data == "quit":
				print("\nquit")
				running = False #beenden
				continue
			
			log.write(data)
				
			
	finally:
		traceback.print_exc(file=sys.stdout)
		try:
			print("closing socket")
			s.close() # close socket
		except:
			print("no clean closing")
		input("press any button to end...")