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
import subprocess # to test crash detection program on plotted data
import matplotlib.gridspec as gridspec  # for subplot organisation
from matplotlib import pyplot as plt

		
		
def getFilename(logFolder="./logs/"):
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
			for root, dirs, files in os.walk(logFolder+"/../", topdown=False):
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


def readData(data_log):
	"""
	Takes the content of a csv file as a list of the lines. 
	With the format of a line beeing: "timestamp(%Y-%m-%d_%H-%M-%S-%f),  descriptor_1, x_1, y_1, z_1,  descriptor_2, x_2, y_2, z_2,  ... \n"
	Returns the data contained in a dict as well as a list of the Datatypes.
	"""
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
	
	# convert data
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
	return data, dataTypes, tOffset

if __name__ == "__main__":
	dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
	t = 't'
	x = 'x'
	y = 'y'
	z = 'z'
	dirname = os.path.dirname(__file__)
	testProgram_rel = "../Optimization/Bikecrasher.exe" if 'w' in sys.platform else "./a.out"
	testProgram = os.path.join(dirname, testProgram_rel)

	# get file of wanted data
	filename = getFilename()
	
	# read files
	with open(filename, "r") as d_log:
		data_log = d_log.readlines()
	with open(filename.replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r") as c_log:
		crash_log = c_log.readlines()

	# prepare and convert data
	data, dataTypes, tOffset = readData(data_log)

	# ask to run detection tool
	addTestrun = "y" in input("Test {} on this data? (y/n)\t".format(testProgram_rel))
	result_t = []
	result_y = []
	result_y_expanded = []
	# run detection tool
	if addTestrun:
		genome_raw = input("Enter threshold parameters seperated by a space:")
		genome = [i.strip() for i in genome_raw.split()]
		arg = [testProgram, filename, *genome] #convert np array to str
		result = subprocess.run(arg, capture_output=True)
		result_str = result.stdout.decode().strip()
		print("\nResult from detection:\n", len(result_str), result_str)
		if result_str:
			# convert output
			for line in result_str.split("\n"):
				split_line = line.split()
				result_t.append((datetime.strptime(split_line[0], dateFormat) - tOffset).total_seconds())
				result_y.append(float(split_line[-1]))
			for time_step in data[t]:
				nothingAdded = True
				for idx, retruned_stamp in enumerate(result_t):
					if retruned_stamp == time_step:
						result_y_expanded.append(result_y[idx])
						nothingAdded = False
				if nothingAdded:
					result_y_expanded.append(0)
		else:
			print("Cannot test {} on this data. Return is empty.".format(testProgram_rel))
			addTestrun = False

	# setup plots
	fig = plt.figure()
	grid = gridspec.GridSpec(ncols=1, nrows=len(dataTypes) + 1*addTestrun)
	
	# plot data
	ax = {} #axis that hold plot
	for idx, dataType in enumerate(dataTypes):
		ax[dataType] = fig.add_subplot(grid[idx,0]) #111)
		ax[dataType].set_title(dataType)
		
		cutExcessIdx_X = min(len(data[t]), len(data[dataType][x]))
		cutExcessIdx_Y = min(len(data[t]), len(data[dataType][y]))
		cutExcessIdx_Z = min(len(data[t]), len(data[dataType][z]))
		
		ax[dataType].plot(data[t][:cutExcessIdx_X], data[dataType][x][:cutExcessIdx_X])
		ax[dataType].plot(data[t][:cutExcessIdx_Y], data[dataType][y][:cutExcessIdx_Y])
		ax[dataType].plot(data[t][:cutExcessIdx_Z], data[dataType][z][:cutExcessIdx_Z])

	# plot detection output
	if addTestrun:
		result_axis = fig.add_subplot(grid[len(dataTypes),0])
		result_axis.set_title("Detection results")
		result_axis.plot(data[t], result_y_expanded)

	# add crash timestamps
	for tstamp in crash_log:
		tsec = (datetime.strptime(tstamp.strip(), dateFormat) - tOffset).total_seconds()
		for dataType in dataTypes:
			ymin = max( max(data[dataType][x]) , max(data[dataType][y]) , max(data[dataType][z]) )
			ymax = min( min(data[dataType][x]) , min(data[dataType][y]) , min(data[dataType][z]) )
			ax[dataType] .vlines(tsec, ymin, ymax, colors='r', linestyles='solid', label='Crash')
		
	plt.show()
	