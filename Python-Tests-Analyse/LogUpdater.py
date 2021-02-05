import os			# file operations
import sys			# get python executable
import copy			# list deep copying
import subprocess	# to call plot script
from datetime import datetime, timedelta # time operations
from shutil import copyfile # to copy timestamps of crash to renamed version

dateFormat = "%Y-%m-%d_%H-%M-%S-%f"
dateFormat_filename = "%Y-%m-%d_%H-%M-%S"
path2logs_rel = "./../Python-Tests-Analyse/logs/"
dirname = os.path.dirname(__file__)
path2logs = os.path.join(dirname, path2logs_rel)

def shiftTimestamps(content, offset):
	content_fixed = copy.deepcopy(content)
	for num, l in enumerate(content):
		if l[0]:
			try:
				lineDate = datetime.strptime(l[0], dateFormat)
			except Exception as err:	
				print("can't read timestamp:", err)
				continue
			else:
				new_lineDate = lineDate + offset
				content_fixed[num][0] = datetime.strftime(new_lineDate, dateFormat)
		else:
			# empty line
			continue
	return content_fixed
	


if __name__ == "__main__":
	# get input files
	if len(sys.argv) > 1:
		print( "{} Dateien:\n{}".format(len(sys.argv)-1, '\n'.join(sys.argv[1:])) )
		files = [os.path.split(path)[-1] for path in sys.argv[1:]]
	else:
		files = os.listdir(path2logs)
	
	# iterate files
	for filename in files:
		# open log file
		with open(os.path.join(path2logs, filename), "r") as f:
			content_lines = f.readlines()
			print(os.path.join(path2logs, filename), "\ncontent:", content_lines[:3])
			content = [[part.strip() for part in line.split(',')] for line in content_lines]
		
		# detect wrong dates
		print(content[:3])
		date1 = datetime.strptime(content[0][0], dateFormat) # first date
		content_fixed = content
		if date1<datetime(2000, 1, 1, 1, 1, 1, 1):
			# timestamps in file are old
			
			# get file date from name
			dateidx = filename.find("_20")
			datestr = filename[dateidx:dateidx+20][1:]
			date_from_name = datetime.strptime(datestr, dateFormat_filename) # len("_2020-12-04_15-29-14") = 20
			# calc offset (first timestamp will equal timestamp in filename)
			offset = date_from_name-date1
			print("file:", filename)
			print("offset:", offset)
			
			content_fixed = shiftTimestamps(content, offset)
		
			# save changes
			with open(os.path.join(path2logs, filename), "w") as f:
				print("Speicherpfad:",os.path.join(path2logs, filename))
				joined1 = [", ".join(splitLines) for splitLines in content_fixed]
				joined2 = "\n".join(joined1)
				f.writelines(joined2)
				print("was gespeichert wird /n",joined2[:5])

		# plot new file
		plot = subprocess.Popen([sys.executable, os.path.join(dirname, "PlotCrash.py"), os.path.join(path2logs, filename)],\
						stdout=subprocess.DEVNULL) #no output
		
		# promt to change crash timestamps
		if 'y' in input("edit timestamp of crash? (y/n)").lower():
			new_timestamps = []
			date1_fixed = datetime.strptime(content_fixed[0][0], dateFormat) # first date in fixed version
			
			with open(os.path.join(path2logs, filename).replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "r+") as f:
				timestamps = f.readlines()
				
				# ask for all crash timestamps
				for num,t in enumerate(timestamps):
					t_date = datetime.strptime(t.strip(), dateFormat)
					delta_t_date = t_date - date1_fixed
					print("Update this timestamp? Enter new value [in seconds] or leave empty for no change.")
					print("\t{}. Stamp at {} sec ({})".format(num+1, delta_t_date.total_seconds(), t.strip()))
					inp = input("\t").strip()
					if inp:
						# update timestamp
						new_delta_t_date = timedelta(seconds=float(inp))
						new_t_date = date1_fixed + new_delta_t_date
						new_t_date_str = datetime.strftime(new_t_date, dateFormat)
						print("New timesstamp is at {} sec ({}).".format(new_delta_t_date.total_seconds(), new_t_date_str))
						new_timestamps.append(new_t_date_str+"\n")
					else:
						new_timestamps.append(t+"\n")
			
			# save new timestamps
			with open(os.path.join(path2logs, filename).replace(".txt", "_timestamps.txt").replace(".csv", "_timestamps.csv"), "w") as f:
				f.writelines(new_timestamps)
		
		# close process
		plot.kill()
		if not 'y' in input("Continue with next file? Will close otherwise. \n\t(\"y\" (continue) / other (close)) :\t").lower():
			break
			
	
