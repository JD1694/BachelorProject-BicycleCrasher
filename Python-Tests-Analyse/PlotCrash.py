"""
General:
Asks for a .txt or .csv file of logged data in the form of: "timestamp(%Y-%m-%d_%H-%M-%S-%f),  descriptor_1, x_1, y_1, z_1,  descriptor_2, x_2, y_2, z_2,  ... \n"
The x,y and z data for each descriptor is plotted over time. x,y and z are not combined into one Graph.
A second file (same name as data file with "_timestamp" appended) is read. It contains timestamps that are marked in the other plots as red lines.
Used for:
Visualizes Data obtained from DataCollection_Client.py. Plots data (accelerometer and orientation calculated with DMP) as well as timestamps of crashes as marked by DataCollection_Client.py.
"""

import sys
import os
from datetime import datetime
import matplotlib.gridspec as gridspec  # for subplot organisation
from matplotlib import pyplot as plt

		
		
def getFilename():
	""" Gets the path to a file. Checks for a commmand line argument and prompts the user otherwise. Entering nothing will get the last modified file from a './logs/' folder. A requested file is searched in all nested folders beginning at the location of this file and fileendings .txt and .cvs may be omitted."""
	# commandline parameters
	if len(sys.argv) > 1:
		print( "{} Dateien:\n{}".format(len(sys.argv)-1, '\n'.join(sys.argv[1:])) )
		filename = sys.argv[1]
	else:
		filename = input("Input CSV-File with Sensor Data. Leave empty to load last modified.\n")
		# find last modified file?
		if not filename.strip():
			lastDate = 0
			lastFile = ""
			# get last modified date of files (in sec since epoche)
			for root, dirs, files in os.walk(".", topdown=False):
				for name in files:
					if name[-4:] == ".txt" and "_timestamp" not in name:
						fileDate = os.path.getmtime(os.path.join(root, name))
						if fileDate > lastDate:
							lastDate = fileDate
							lastFile = os.path.join(root, name)
			filename = lastFile
			print("Opening last modified File: ", os.path.basename(filename))
			
		# verify and correct given file
		elif not os.path.isfile(filename):
			nameWOpath = os.path.basename(filename)
			logFolder = "./logs/"
			# correct folder
			if os.path.isfile(logFolder + nameWOpath):
				filename = logFolder + nameWOpath
			# correct file-ending
			elif os.path.isfile(filename + ".txt"):
				filename = filename + ".txt"
			elif os.path.isfile(filename + ".csv"):
				filename = filename + ".csv"
			# correct both
			elif os.path.isfile(logFolder + nameWOpath + ".txt"):
				filename = logFolder + nameWOpath + ".txt"
			elif os.path.isfile(logFolder + nameWOpath + ".csv"):
				filename = logFolder + nameWOpath + ".csv"
			# no match, restart
			else:
				input("File was not found! Please enter a valid .csv or .txt log file. ")
				return getFilename()
				
	return filename


def isfloat(value):
	"""Returns True if String can be converted into a float and False if not."""
	try:
		float(value)
		return True
	except ValueError:
		return False


if __name__ == "__main__":
	
	# get file of wanted data
	filename = getFilename()
	
	# read files
	with open(filename, "r") as d_log:
		data_log = d_log.readlines()
	with open(filename.replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r") as c_log:
		crash_log = c_log.readlines()

	# prepare data
	while '\n' in data_log:
		data_log.remove('\n')
	while '' in data_log:
		data_log.remove('')
		
	# ready data storage in RAM
	# get dataTypes out of first line in log (excluding timestamp at first position)
	dataTypes = [element.strip() for element in data_log[0].split(',')[1:] if not isfloat(element.strip()) ] #['areal', 'ypr', 'accel', 'gyro']
	print("Data Types:", dataTypes)
	t = 't'
	x = 'x'
	y = 'y'
	z = 'z'
	data = {t:[]} # Final Format:  {t:[], ypr:{x:[], y:[], z:[]}, areal:{x:[], y:[], z:[]}, ...}
	for dataType in dataTypes:
		data[dataType] = {x:[], y:[], z:[]}
	dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
	
	# convert read date
	partLine = ""
	tOffset = None
	lineLength = len(data_log[0].split(','))
	for raw_line in data_log:
		try:
			line = raw_line.split(',')
			# attempt to merge lines that were split
			if len(line)<lineLength:
				print("Can't process line of length:", len(line), raw_line)
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
			idx_in_line = 2
			for dataType in dataTypes:
				data[dataType][x].append(float(line[idx_in_line]))
				data[dataType][y].append(float(line[idx_in_line+1]))
				data[dataType][z].append(float(line[idx_in_line+2]))
				idx_in_line += 4 # offset 3 values and 1 title

		except Exception as error:
			print(error, "WITH LINE:", raw_line)

	# setup plots
	fig = plt.figure()
	grid = gridspec.GridSpec(ncols=1, nrows=len(dataTypes))
	
	# plot data
	ax = {} #axis that hold plot
	for idx, dataType in enumerate(dataTypes):
		ax[dataType] = fig.add_subplot(grid[idx,0]) #111)
		ax[dataType].set_title(dataType)
		
		ax[dataType].plot(data[t], data[dataType][x])
		ax[dataType].plot(data[t], data[dataType][y])
		ax[dataType].plot(data[t], data[dataType][z])

	# add crash timestamps
	for tstamp in crash_log:
		tsec = (datetime.strptime(tstamp.strip(), dateFormat) - tOffset).total_seconds()
		for dataType in dataTypes:
			ymin = max( max(data[dataType][x]) , max(data[dataType][y]) , max(data[dataType][z]) )
			ymax = min( min(data[dataType][x]) , min(data[dataType][y]) , min(data[dataType][z]) )
			ax[dataType] .vlines(tsec, ymin, ymax, colors='r', linestyles='solid', label='Crash')
		
	plt.show()