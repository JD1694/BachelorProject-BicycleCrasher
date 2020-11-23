import Serverconnect # Connect to Sensor Device
import sys
import traceback
import keyboard  # hotkeys ???
from datetime import datetime

def callback_button(stamp_log):
	timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")
	print("\n###  CRASH AT: " + timestamp + "  ###\n\n")
	stamp_log.write(timestamp + "\n")
	

if __name__ == "__main__":
	try:
		# choose file to save to
		logname = "logs/dataLog_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
		log = open(logname + ".txt", "w")
		stamp_log = open(logname + "_timestamps" + ".txt", "w")
		
		# setup hotkeys
		keyboard.add_hotkey('space', callback_button, args=[stamp_log])
			
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
		try:
			print("closing log file")
			stamp_log.close()
		except:
			print("no clean closing log file")
			
		input("press any button to end...")