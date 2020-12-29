import sys
import os
from datetime import datetime
import random
import subprocess



iterations = 3		# number of random files to test
timeOffset = 20	# delta seconds that calculated timestamp of crash may vary from recorded timestamp.
punishment = -1		# negative value added rating for false positives
testProgram = "./Bikecrasher.exe"
dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
rating = None
timestampReturned = None
timestampsExpected = None
timestampOffsetCorrection = 0
score = 0


def pointsByTime(tDiff):
	"""
	A function that grants many point if the difference is negative (crash detected before timestamped)
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


if __name__ == "__main__":
	### get genome from evolutionary algorithm
	genome = sys.argv[1:]
	
	### choose files
	path2logs = "./../Python-Tests-Analyse/logs/"
	filelist = [os.path.join(path2logs, file) for file in os.listdir(path2logs) if not "_timestamps" in file]
	random.shuffle(filelist)
	
	for filename in filelist[:iterations]:
	
		### call program to evaluate
		result = subprocess.run([testProgram, filename, *genome], capture_output=True)
		result_str = result.stdout.decode().strip()
		
		if result_str:
			timestampReturned = datetime.strptime(result_str, dateFormat)
		else:
			# no timestamp was returned
			timestampReturned = None
		
		### calc rating
		# read correct value
		with open(filename.replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r") as solutionFile:
			solutions = solutionFile.readlines()
		if solutions:
			if timestampReturned:
				# crash was logged
				timestampsExpected = [datetime.strptime(line.strip(), dateFormat) for line in solutions]
				nearestExpectedTimestamp = min(timestampsExpected, key=lambda t: abs(timestampReturned - t).total_seconds())
				ratingCurrentFile = pointsByTime((timestampReturned - nearestExpectedTimestamp).total_seconds())
				print("Crash was detected. Right! \t Rating:", ratingCurrentFile, "\t\t", os.path.basename(filename))
			else:
				ratingCurrentFile = punishment
				print("No crash detected. Wrong! \t Rating:", ratingCurrentFile, "\t\t", os.path.basename(filename))
		else:
			# log is of clean driving
			if timestampReturned:
				ratingCurrentFile = punishment
				print("Crash was detected. Wrong! \t Rating:", ratingCurrentFile, "\t\t", os.path.basename(filename))
			else:
				ratingCurrentFile = -5 * punishment
				print("No crash detected. Right! \t Rating:", ratingCurrentFile, "\t\t", os.path.basename(filename))
		score += ratingCurrentFile

		
	### return final rating to evolutionary algorithm
	print(score/iterations)
