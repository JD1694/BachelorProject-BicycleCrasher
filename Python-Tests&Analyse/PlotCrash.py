"""
Visualizes Data obtained from DataCollection_Client.py. Plots data (accelerometer and orientation calculated with DMP) as well as timestamps of crashes as marked by DataCollection_Client.py.
"""


import matplotlib.gridspec as gridspec  # for subplot organisation
from matplotlib import pyplot as plt
from datetime import datetime
import sys



		
		
if __name__ == "__main__":
	areal = 'areal'
	ypr = 'ypr'
	t = 't'
	x = 'x'
	y = 'y'
	z = 'z'
	data = {t:[], ypr:{x:[], y:[], z:[]}, areal:{x:[], y:[], z:[]}}
	dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
	
	# Kommandozeilen Argumente
	if len(sys.argv) > 1:
		print( "{} Dateien:\n{}".format(len(sys.argv)-1, '\n'.join(sys.argv[1:])) )
		filename = sys.argv[1]
	else:
		filename = input("Input CSV-File with Sensor Data:")
	
	# read files
	with open(filename, "r") as d_log:
		data_log = d_log.readlines()
	with open(filename.split(".")[0] + "_timestamps." + filename.split(".")[-1], "r") as c_log:
		crash_log = c_log.readlines()

	# prepare data
	while '\n' in data_log:
		data_log.remove('\n')
	while '' in data_log:
		data_log.remove('')
	
	# convert read date
	partLine = ""
	tOffset = None
	for raw_line in data_log:
		try:
			line = raw_line.split(',')
			# attempt to merge lines that were split
			if len(line)<9:
				print("len(line)", len(line), raw_line)
				if partLine:
					line = partLine.split(',') + raw_line.split(',')
				else:
					partLine = raw_line
					continue
			else:
				partLine = ""
			
			# offset time to start at 0
			if tOffset:
				data[t].append((datetime.strptime(line[0], dateFormat) - tOffset).total_seconds())
			else:
				data[t].append(0.0)
				tOffset = datetime.strptime(line[0], dateFormat)
			
			# convert and store
			data[ypr][x].append(float(line[2]))
			data[ypr][y].append(float(line[3]))
			data[ypr][z].append(float(line[4]))
			data[areal][x].append(int(line[6]))
			data[areal][y].append(int(line[7]))
			data[areal][z].append(int(line[8]))
		except Exception as error:
			print(error, "WITH LINE:", raw_line)

	# setup plots
	fig = plt.figure()
	grid = gridspec.GridSpec(ncols=1, nrows=2)
	ax1 = fig.add_subplot(grid[0,0]) #111)
	ax2 = fig.add_subplot(grid[1,0]) #212)
	
	ax1.set_title(ypr)
	ax2.set_title(areal)
	
	# plot data
	ax1.plot(data[t], data[ypr][x])
	ax1.plot(data[t], data[ypr][y])
	ax1.plot(data[t], data[ypr][z])
	ax2.plot(data[t], data[areal][x])
	ax2.plot(data[t], data[areal][y])
	ax2.plot(data[t], data[areal][z])

	# add crash timestamps
	ymin_ax1 = max( max(data[ypr][x]) , max(data[ypr][y]) , max(data[ypr][z]) )
	ymax_ax1 = min( min(data[ypr][x]) , min(data[ypr][y]) , min(data[ypr][z]) )
	ymin_ax2 = max( max(data[areal][x]) , max(data[areal][y]) , max(data[areal][z]) )
	ymax_ax2 = min( min(data[areal][x]) , min(data[areal][y]) , min(data[areal][z]) )
	for tstamp in crash_log:
		tsec = (datetime.strptime(tstamp.strip(), dateFormat) - tOffset).total_seconds()
		ax1.vlines(tsec, ymin_ax1, ymax_ax1, colors='r', linestyles='solid', label='Crash')
		ax2.vlines(tsec, ymin_ax2, ymax_ax2, colors='r', linestyles='solid', label='Crash')
		
	plt.show()