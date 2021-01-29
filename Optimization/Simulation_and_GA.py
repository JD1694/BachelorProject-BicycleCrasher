import sys
import os
from datetime import datetime
import random
import subprocess
import ast  # convert Strings

import numpy as np
from geneticalgorithm import geneticalgorithm as ga



# Constants: configuration and paths
timeOffset = 2	# delta seconds that calculated timestamp of crash may vary from recorded timestamp.
punishment = -100	# negative value added rating for false positives
dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
testProgram_rel = "./Bikecrasher.exe" if 'w' in sys.platform else "./a.out"
path2logs_rel = "./../Python-Tests-Analyse/logs/"

normalDrivingLog1_rel = "normalesfahren_2020-12-18_17-51-07.txt" # against false positives
normalDrivingLog2_rel = "normales_fahren_easy_2021-01-08_17-48-25.txt"
crashLog1_rel = "aauslaufen_2020-12-18_17-21-33.txt" # against false negatives
crashLog2_rel = "jdtrittfahrradvonvorne_2020-12-18_17-44-33.txt"
crashLog3_rel = "trittgegenhinterrad_2020-12-18_17-46-50.txt"
crashLog4_rel = "schraegdenrasenrunter_2020-12-18_17-48-35.txt"
#treppe_hoch_2020-12-18_17-25-07.txt
#crashLog1_rel = "auslaufen_2020-12-18_17-07-38.txt"
#crashLog2_rel = "auslaufen_2020-12-18_17-11-11.txt"
#crashLog3_rel = "wegziehen_2020-12-18_17-58-16.txt"
#crashLog4_rel = "wegziehen_2020-12-18_17-59-22.txt"

# make file paths independant of working directory
dirname = os.path.dirname(__file__)
path2logs = os.path.join(dirname, path2logs_rel)
testProgram = os.path.join(dirname, testProgram_rel)
normalDrivingLog1 = os.path.join(path2logs, normalDrivingLog1_rel)
normalDrivingLog2 = os.path.join(path2logs, normalDrivingLog2_rel)
crashLog1 = os.path.join(path2logs, crashLog1_rel)
crashLog2 = os.path.join(path2logs, crashLog2_rel)
crashLog3 = os.path.join(path2logs, crashLog3_rel)
crashLog4 = os.path.join(path2logs, crashLog4_rel)




def pointsByTime(tDiff):
	"""
	A function that grants many points if the difference is negative (crash detected before timestamped)
	and less points the bigger the difference gets. 
	But none, if it is outside a maximum timeOffset where it is no longer considerd a crash.
			 +
			 +  +
		   y=0     +
			 +        +
	------------t=0------------------> t
	"""
	if abs(tDiff) < timeOffset:
		return -tDiff + timeOffset + punishment
	else:
		return punishment
		

def antiFalsePositive(secondsLeft):
	""" Used in case of a log file without a crash. Generally negative when a crash is detected nevertheless. 
	More points are given, the later the false positive is made. 
	Param: seconds left from timestamp of detecetion until EOF (End Of File)"""
	return -secondsLeft


def runSimulation(genome, debug=0):
	# used vars
	
	rating = None
	timestampReturned = None
	timestampsExpected = None
	score = 0
	
	### get genome from evolutionary algorithm as numpy array
	printDebug(debug, "\n")
	printDebug(debug, genome)

	# most log files are flawed, use manual chosen files instead
	filelist = [crashLog1, crashLog2, crashLog3, crashLog4, normalDrivingLog1, normalDrivingLog2]

	
	for filename in filelist:
	
		### call program to evaluate
		arg = [testProgram, filename, *genome.astype(str)] #convert np array to str
		result = subprocess.run(arg, capture_output=True)
		printDebug(debug-1, result)
		result_str = result.stdout.decode().strip()
		printDebug(debug, result_str)
		
		if result_str:
			#timestampReturned = datetime.strptime(result_str, dateFormat)
			timeWithPropability = [[datetime.strptime(line.split(" ")[0], dateFormat), float(line.split(" ")[-1])] for line in result_str.split("\n") if line]
		else:
			# no timestamp was returned
			#timestampReturned = None
			timeWithPropability = None
		
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
		
		ratingCurrentFile = 0
		if solutions:
			if timeWithPropability: #timestampReturned:
				# crash was logged
				#nearestExpectedTimestamp = min(timestampsExpected, key=lambda t: abs(timestampReturned - t).total_seconds())
				#ratingCurrentFile = pointsByTime((timestampReturned - nearestExpectedTimestamp).total_seconds())
				#printDebug(debug-2, "\nRight! Crash was detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, nearestExpectedTimestamp, os.path.basename(filename)))
				
				for timestampReturned, percent in timeWithPropability:
					# get nearest crash
					nearestExpectedTimestamp = min(timestampsExpected, key=lambda t: abs(timestampReturned - t).total_seconds())
					# rate this stamp mith percentage confidence
					ratingCurrentFile += pointsByTime((timestampReturned - nearestExpectedTimestamp).total_seconds()) * percent
				printDebug(debug-2, "\nRight! Crash was detected.\t Rating: {}\t\t returned: {} length: {} file: {}\n".format(ratingCurrentFile, timeWithPropability[1], len(timeWithPropability), os.path.basename(filename)))
			else:
				ratingCurrentFile = punishment
				printDebug(debug-2, "\nWrong! No crash detected.\t Rating: {}\t\t returned: {} expected: {} file: {}\n".format(ratingCurrentFile, None, ", ".join(solutions), os.path.basename(filename)))
		else:
			# log is of clean driving
			if timeWithPropability: #timestampReturned:
				#ratingCurrentFile = antiFalsePositive((timestamp_EOF - timestampReturned).total_seconds())
				#printDebug(debug-2, "\nWrong! Crash was detected.\t Rating: {}\t\t returned: {} expected: {} \t file: {}\n".format(ratingCurrentFile, timestampReturned, "None", os.path.basename(filename)))
				
				for timestampReturned, percent in timeWithPropability:
					ratingCurrentFile += antiFalsePositive((timestamp_EOF - timestampReturned).total_seconds()) * percent
				printDebug(debug-2, "\nWrong! Crash was detected.\t Rating: {}\t\t returned: {} expected: {} file: {}\n".format(ratingCurrentFile, timestampReturned, "None", os.path.basename(filename)))
			else:
				ratingCurrentFile = -5 * punishment
				printDebug(debug-2, "\nRight! No crash detected.\t Rating: {}\t\t returned: {} expected: {} file: {}\n".format(ratingCurrentFile, None, None, os.path.basename(filename)))

		printDebug(debug-1, ratingCurrentFile)
		score += ratingCurrentFile

		
	### return final rating to evolutionary algorithm
	printDebug(score/len(filelist))
	return -score/len(filelist)


def printDebug(debug, *strings):
	if debug>=1:
		print(*strings)


def runGA(generations=15, populationSize=30):
	varbound=np.array([[0,1000]]*13)

	algorithm_param = {'max_num_iteration':generations,\
					   'population_size':populationSize,\
					   'mutation_probability':0.15,\
					   'elit_ratio': 0.01,\
					   'crossover_probability': 0.5,\
					   'parents_portion': 0.3,\
					   'crossover_type':'uniform',\
					   'max_iteration_without_improv':20}

	model=ga(function=runSimulation,\
				dimension=13,\
				variable_type='real',\
				variable_boundaries=varbound,\
				algorithm_parameters=algorithm_param)

	model.run()


if __name__ == "__main__":
	if sys.argv[1:]:
		print("Running simulation once with given genome")
		print(runSimulation(np.array(sys.argv[1:]), debug=3))
	else:
		print("starting Genetic Algorithm")
		inpPopulation = int(input("Enter Population Size: "))
		inpGeneration = int(input("Enter Number of Generations: "))
		runGA(generations=inpGeneration, populationSize=inpPopulation)
	input("\nDone. Press enter to exit...")