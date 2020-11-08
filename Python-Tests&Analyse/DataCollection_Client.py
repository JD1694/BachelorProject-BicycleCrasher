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
		running = True
		while running:
			# recieve Data
			if UDP_enable:
				data = Serverconnect.gettingUDPtext(s)
			else:
				data = Serverconnect.gettingtext(client)
			
			if data:
				# stop condition
				print("data: ", data)
				if data == "done"or data == "quit":
					print("\nquit")
					running = False #beenden
					continue
				
				# write data to log file
				log.write(data)
				log.write("\n")
				
			
	finally:
		traceback.print_exc(file=sys.stdout)
		try:
			print("closing socket")
			s.close()
		except:
			print("no clean closing socket")
		try:
			print("closing log file")
			log.close()
		except:
			print("no clean closing log file")
			
		input("press any button to end...")