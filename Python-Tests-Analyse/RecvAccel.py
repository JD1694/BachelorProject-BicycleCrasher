import multiprocessing
import Serverconnect # Connect to Sensor Device
import TiltFilter # conversion from Accelerometer Data to angles
import PlotSensors # Plot Information
from matplotlib import animation
from matplotlib import pyplot as plt
import sys, time, traceback

def startPlot(showtime, pipeOut):
	print("startPlot here")
	ani = animation.FuncAnimation(showtime.fig, showtime.update, fargs=(pipeOut,), \
									  frames=None, interval=100)
	plt.show()
	print("startPlot out")

if __name__ == "__main__":
	try:
		# choose connection type (UDP?)
		UDP_enable = Serverconnect.chooseType()
	
		# setup for plot animation
		showtime = PlotSensors.PlotRunning() if input("Choose plot type. 3D (type \"1\") or graph (default):").strip() == "1" else PlotSensors.PlotLog()
	
		# setup Angle calculation
		print("creating Complementary Filter")
		Filter = TiltFilter.ComplementaryFilter()
			
		# New Process for GUI
		print("creating Pipe")
		pipeOut, pipeIn = multiprocessing.Pipe()  # makes communication possible
		print("creating Process")
		plotWindow = multiprocessing.Process(target=startPlot, args=(showtime, pipeOut))#daemon=True?
		print("starting Process")
		plotWindow.start()

		# Communikation with Android
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
			
			# enhance input: make floats, filter unnessessary
			try:
				# Convert Data to list with numbers
				#data = eval(data)
				#z.B. data: "47071.57028, 3,   0.139, -0.194,  9.732, 4,   0.002,  0.024,  0.000, 5, -31.073,-20.804,-44.763"
				data_list = [float(d.strip()) for d in data.split(',')]
				data_dict = {'Accelerometer': data_list[2:5], 'Gyroscope': data_list[6:9]}
			
			# check values
			except:
				traceback.print_exc(file=sys.stdout)
				print("ISSUE1:", data)
			if type(data_list[0]) != float and type(data_list[0]) != int:
				input("ISSUE2: failed to convert sensor readings to numbers", data)
			elif len(data_dict['Accelerometer']) != 3 or len(data_dict['Gyroscope']) != 3:
				print("ISSUE3: not all sensor readings available", data)
				continue
			
			else:
				# Convert Accelerometer Data to Angles
				
				#data_list = [-x for x in data_list] # invert sign
				result['Calculated Angle'] = Filter.calcOrientation(data_dict['Gyroscope'], data_dict['Accelerometer'])
				result['Accelerometer'] = Filter.angleFromAccel(*data_dict['Accelerometer'])
				result['Gyroscope'] = Filter.angleFromGyro_integrationOnly(*data_dict['Gyroscope'])
				
				### if not firstReading:
					### print("\b"*33, end='', flush=True)
				### else:
					### firstReading = False
				### print("x: {:06.2f}   y: {:06.2f}   z: {:06.2f}".format(x, y, z), end='')
				
				# Pack and send Data to Plot Window
				pipeIn.send(result)
				
				
	finally:
		traceback.print_exc(file=sys.stdout)
		try:
			print("closing...")
			multiprocessing.active_children() # entzombifies inactive processes. 
			print("closed 1")
			s.close() # close socket
			print("closed 2")
		except:
			print("no clean closing")
		input("beenden...")