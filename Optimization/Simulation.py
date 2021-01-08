import sys
import os
from datetime import datetime
import random
import subprocess
import ast  # convert Strings



# Constants: configuration and paths
iterations = 3		# number of random files to test
timeOffset = 7	# delta seconds that calculated timestamp of crash may vary from recorded timestamp.
punishment = -1		# negative value added rating for false positives
dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
testProgram_rel = "./Bikecrasher.exe"
path2logs_rel = "./../Python-Tests-Analyse/logs/"

normalDrivingLog_rel = "normalesfahren_2020-12-18_17-51-07.txt" # only file of this kind
crashLog1_rel = "auslaufen_2020-12-18_17-07-38.txt"
crashLog2_rel = "auslaufen_2020-12-18_17-11-11.txt"

# make file paths independant of working directory
dirname = os.path.dirname(__file__)
path2logs = os.path.join(dirname, path2logs_rel)
testProgram = os.path.join(dirname, testProgram_rel)
normalDrivingLog = os.path.join(path2logs, normalDrivingLog_rel)
crashLog1 = os.path.join(path2logs, crashLog1_rel)
crashLog2 = os.path.join(path2logs, crashLog2_rel)

# used vars
debug = False
rating = None
timestampReturned = None
timestampsExpected = None
timestampOffsetCorrection = 0
score = 0




def pointsByTime(tDiff):
	"""
	A function that grants many points if the difference is negative (crash detected before timestamped)
	and less points the bigger the difference gets. 
	But none, if it is outside a maximum timeOffset where it is no longer considerd a crash.
			 +
			 +  +
			 +     +
			 +        +
	--------------------------------> t
	"""
	if abs(tDiff) < timeOffset:
		return -tDiff + timeOffset + punishment
	else:
		return punishment
		

def antiFalsePositive(secondsLeft):
	""" Used in case of a log file without a crash. Generally negative when a crash is detected nevertheless. 
	More points are given, the later the false positive is made. 
	Param: seconds left from timestamp of detecetion until EOF (End Of File)"""
	return -100 * secondsLeft


def printDebug(debug, *strings):
	if debug:
		print(*strings)


if __name__ == "__main__":
	### get genome from evolutionary algorithm
	if "debug" in sys.argv:
		debug = True
		print("DEBUG Mode activated")
	genome = sys.argv[-1] if sys.argv[1:] else '[' + ', '.join(['1' for i in range(20)]) + ']'
	genome = ast.literal_eval(genome) # list(genome)
	genome = [str(i) for i in genome]
	printDebug(debug, type(genome), genome)
	
	
	### choose files
	##filelist = [os.path.join(path2logs, file) for file in os.listdir(path2logs) if not "_timestamps" in file]
	##random.shuffle(filelist)
	##filelist = filelist[:iterations]
	##filelist.append(normalDrivingLog)
	
	# most log files are flawed, use manual chosen files instead
	filelist = [crashLog1, crashLog2, normalDrivingLog]
	
	
	for filename in filelist:
	
		### call program to evaluate
		result = subprocess.run([testProgram, filename, *genome], capture_output=True)
		printDebug(debug, result)
		result_str = result.stdout.decode().strip()
		
		if result_str:
			timestampReturned = datetime.strptime(result_str, dateFormat)
		else:
			# no timestamp was returned
			timestampReturned = None
		
		### calc rating
		# read correct value
		solutions = None
		timestampsExpected = None
		with open(filename.replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r") as solutionFile:
			solutions = solutionFile.readlines()
			timestampsExpected = [datetime.strptime(line.strip(), dateFormat) for line in solutions]
		with open(filename, "r") as logFile:
			lines = logFile.readlines()
			for l in lines[::-1]:
				line_parts = l.split(',')
				if len(line_parts) > 1:
					timestamp_EOF = datetime.strptime(line_parts[0].strip(), dateFormat)
					break
		
		if solutions:
			if timestampReturned:
				# crash was logged
				nearestExpectedTimestamp = min(timestampsExpected, key=lambda t: abs(timestampReturned - t).total_seconds())
				ratingCurrentFile = pointsByTime((timestampReturned - nearestExpectedTimestamp).total_seconds())
				printDebug(debug, "\nRight! Crash was detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, nearestExpectedTimestamp, os.path.basename(filename)))
			else:
				ratingCurrentFile = punishment
				printDebug(debug, "\nWrong! No crash detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, ", ".join(solutions), os.path.basename(filename)))
		else:
			# log is of clean driving
			if timestampReturned:
				ratingCurrentFile = antiFalsePositive((timestamp_EOF - timestampReturned).total_seconds())
				printDebug(debug, "\nWrong! Crash was detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, "None", os.path.basename(filename)))

			else:
				ratingCurrentFile = -5 * punishment
				printDebug(debug, "\nRight! No crash detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, "None", os.path.basename(filename)))

		score += ratingCurrentFile

		
	### return final rating to evolutionary algorithm
	print(score/iterations)
