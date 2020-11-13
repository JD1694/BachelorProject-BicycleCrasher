import Serverconnect # Connect to Sensor Device
import sys
import traceback
import keyboard  # hotkeys ???
from datetime import datetime

def callback_button(log):
	timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	stampStr = "\n###  CRASH AT: " + timestamp + "  ###\n\n"
	print(stampStr)
	log.write(stampStr)
	

if __name__ == "__main__":
	try:
		# choose file to save to
		logname = "logs/dataLog_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".txt"
		log = open(logname, "w")
		
		# setup hotkeys
		keyboard.add_hotkey('space', callback_button, args=[log])
			
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