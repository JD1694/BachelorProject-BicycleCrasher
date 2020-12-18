"""

"""
import math
import sys
import os
from datetime import datetime


def rotateZ(angle, point):
	"""
	Rotate a point around the Z-Axis by the angle phi in degrees. Returns the rotated point as a tuple. Works with both 2D and 3D points.
	"""
	radAngle = math.radians(angle)
	x = math.cos(radAngle)*point[0] - math.sin(radAngle)*point[1]
	y = math.sin(radAngle)*point[0] + math.cos(radAngle)*point[1]
	return (x, y, point[2]) if len(point)==3 else (x, y)


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
	return data, dataTypes


def isfloat(value):
	"""Returns True if String can be converted into a float and False if not."""
	try:
		float(value)
		return True
	except ValueError:
		return False

if __name__ == "__main__":
	x, y, z = 'x', 'y', 'z'
	angleResolution = 10
	# get file of wanted data
	filename = getFilename(logFolder="../Python-Tests-Analyse/logs/")
	
	# read calibration file with known rotation around the y-axis we want
	with open(filename, "r") as f:
		data_log = f.readlines()
	
	# prepare and convert data
	data, dataTypes = readData(data_log)

	# average gyro
	gyroVec = {x:0, y:0, z:0}
	for xyz in ['x', 'y', 'z']:
		gyroVec[xyz] = sum([abs(value) for value in data['gyro'][xyz]])/len(data['gyro'][xyz])
	print("average of all gyro readings in file: ", gyroVec)
	
	# try multiple rotation angles
	bestPhi, bestResult = None, -math.inf
	for phi in [p/angleResolution for p in range(0, 360*angleResolution)]:
		# rotate input
		rotGyroVec = rotateZ(phi, (gyroVec[x], gyroVec[y], gyroVec[z]))
		ratioY = abs(rotGyroVec[1])/(abs(rotGyroVec[0])+abs(rotGyroVec[2]))
		print("phi: {:5.1f}, rotated Y: {:7.2f}, rotated Y ratio: {:5.2f}, \t".format(phi, rotGyroVec[1], ratioY), "#"*int(rotGyroVec[1]*10))
		# test result
		if ratioY > bestResult:
			bestPhi, bestResult = phi, ratioY
			print("  # better than last best")
	
	print("\nBest angle of rotation about Z-Axis to maximise Y Komponent: ", bestPhi)
	input("Press enter to end programm...")