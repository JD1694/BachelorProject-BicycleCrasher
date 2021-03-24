"""

"""
import math
import sys
import os
from datetime import datetime
from matplotlib import pyplot, gridspec # plot results from rotation


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
	x, y, z, t = 'x', 'y', 'z', 't'
	angleResolution = 5
	plotWidth = 50 # ca terminal width / 4
	# get file of wanted data
	filename = getFilename(logFolder="../Python-Tests-Analyse/logs/")
	
	# read calibration file with known rotation around the y-axis we want
	with open(filename, "r") as f:
		data_log = f.readlines()
	
	# prepare and convert data
	data, dataTypes = readData(data_log)

	# average gyro, get max & min values
	gyroVec = {x:0, y:0, z:0}  # when calibrated with max y: {x:min, y:max, z:min}
	gyroVec_max = {x:0, y:0, z:0}
	gyroVec_min = {x:0, y:0, z:0}
	for xyz in [x, y, z]:
		gyroVec[xyz] = sum([abs(value) for value in data['gyro'][xyz]]) / len(data['gyro'][xyz])
		gyroVec_max[xyz] = max(data['gyro'][xyz])
		gyroVec_min[xyz] = min(data['gyro'][xyz])
	print("average of all gyro readings in file: ", gyroVec)
	
	# try multiple rotation angles
	bestPhi, bestPhi_Result = None, -math.inf
	for phi in [p/angleResolution for p in range(0, 360*angleResolution)]:
		# rotate input
		rotGyroVec = rotateZ(phi, (gyroVec[x], gyroVec[y], gyroVec[z]))
		ratioYX = abs(rotGyroVec[1])/(abs(rotGyroVec[0])) # don't look at Z. rotation around Z does not change Z value
		
		#print("phi: {:5.1f}, rotated Y: {:7.2f}, rotated Y ratio: {:5.2f}, \t".format(phi, rotGyroVec[1], ratioYX), "#"*int(rotGyroVec[1]*10))
		#dispLenX = int(  (rotGyroVec[0]-gyroVec_min[x]) / (gyroVec_max[x]-gyroVec_min[x]) * plotWidth  )
		#dispLenY = int(  (rotGyroVec[1]-gyroVec_min[y]) / (gyroVec_max[y]-gyroVec_min[y]) * plotWidth  )
		#dispLenZ = int(  (rotGyroVec[2]-gyroVec_min[z]) / (gyroVec_max[z]-gyroVec_min[z]) * plotWidth  )
		dispLenX = int(  rotGyroVec[0] / (gyroVec_max[x]-gyroVec_min[x]) * plotWidth  )
		dispLenY = int(  rotGyroVec[1] / (gyroVec_max[y]-gyroVec_min[y]) * plotWidth  )
		dispLenZ = int(  rotGyroVec[2] / (gyroVec_max[z]-gyroVec_min[z]) * plotWidth  )
		print("phi: {:5.1f} \t".format(phi), end='  |  ')
		print("#"*dispLenX if dispLenX>0 else "-"*abs(dispLenX), " "*(plotWidth-abs(dispLenX)), end='  |  ')
		print("#"*dispLenY if dispLenY>0 else "-"*abs(dispLenY), " "*(plotWidth-abs(dispLenY)), end='  |  ')
		print("#"*dispLenZ if dispLenZ>0 else "-"*abs(dispLenZ))
		
		# test result
		if ratioYX > bestPhi_Result:
			bestPhi, bestPhi_Result = phi, ratioYX
			#print("  # better than last best")
	
	print("\nBest angle of rotation about Z-Axis to maximise Y Komponent: ", bestPhi)
	
	if "y" in input("plot log file with rotation from result? (y/n)"):
		# setup plots
		fig = pyplot.figure()
		grid = gridspec.GridSpec(ncols=2, nrows=len(dataTypes))
		
		# plot data
		ax = {} #axis that hold plot
		for idx_lr, old_new in enumerate(["raw data", "rotated by "+str(bestPhi)+" degrees"]):
			ax[old_new] = {}
			for idx, dataType in enumerate(dataTypes):
				ax[old_new][dataType] = fig.add_subplot(grid[ idx, idx_lr ]) #111
				ax[old_new][dataType].set_title(dataType+' - '+old_new)
				
				if idx_lr:
					# rotated part
					rotData = {x:[], y:[], z:[]}
					for i in range(min( len(data[dataType][x]), len(data[dataType][y]), len(data[dataType][z]) )):
						if dataType == "ypr":
							# YPR coordsys != xyz coordsys. Rotate around Yaw
							rotPoint = rotateZ(bestPhi, (data[dataType][z][i], data[dataType][y][i], data[dataType][x][i]))
							rotPoint = list(reversed(rotPoint))
						else:
							rotPoint = rotateZ(bestPhi, (data[dataType][x][i], data[dataType][y][i], data[dataType][z][i]))
						rotData[x].append(rotPoint[0])
						rotData[y].append(rotPoint[1])
						rotData[z].append(rotPoint[2])

					ax[old_new][dataType].plot(rotData[x], label=x if dataType!="ypr" else "Yaw")
					ax[old_new][dataType].plot(rotData[y], label=y if dataType!="ypr" else "Pitch")
					ax[old_new][dataType].plot(rotData[z], label=z if dataType!="ypr" else "Roll")
				else:
					ax[old_new][dataType].plot(data[dataType][x], label=x if dataType!="ypr" else "Yaw")
					ax[old_new][dataType].plot(data[dataType][y], label=y if dataType!="ypr" else "Pitch")
					ax[old_new][dataType].plot(data[dataType][z], label=z if dataType!="ypr" else "Roll")
				ax[old_new][dataType].legend(loc='upper left')
				#ax[old_new][dataType].set_xlabel("t in [s]")
		
		pyplot.subplots_adjust(hspace=0.4)
		pyplot.show()
	
	input("Press enter to end programm...")