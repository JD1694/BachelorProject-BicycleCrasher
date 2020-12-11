import sys
import os
import datetime
import random
import subprocess



iterations = 3
timeOffset = 10
testProgram = "./testProgram.out"
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
		return -tDiff + timeOffset
	else:
		return 0.0


if __name__ == "__main__":
	### get genome from evolutionary algorithm
	genome = sys.argv[:]
	
	### choose files
	path2logs = "./logs/"
	filelist = os.listdir(path2logs)
	random.shuffle(filelist)
	
	for filename in filelist[:iterations]:
		### call program to evaluate
		result = subprocess.run([testProgram, filename, *genome], capture_output=True)
		timestampReturned = datetime.strptime(result.stdout.strip(), dateFormat)
		print("Crash detected at:", result.stdout.strip())
		
		### calc rating
		# read correct value
		with open(filename.replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r") as solutionFile:
			timestampsExpected = [datetime.strptime(line.strip(), dateFormat) for line in solutionFile.readlines()]
		# calc difference
		nearestExpectedTimestamp = min(timestampsExpected, key=lambda t: abs(timestampReturned - t).total_seconds())
		score += pointsByTime(timestampReturned - nearestExpectedTimestamp).total_seconds())
		
		
		
	### return final rating to evolutionary algorithm
	print(score/iterations)
