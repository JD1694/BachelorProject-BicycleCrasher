import sys
from math import atan, sqrt, degrees, pi


class ComplementaryFilter:
	""" 
	Calculates the Orientation of a IMU-Sensor as angles in rad. 
	Combines accelerometer and gyroscope data using a Complementary Filter to get robust Angle calculations. 
	Use calcOrientation() for filtered Data or other functions to visualize inner working.
	"""
	
	def __init__(self, dt=0.1, A_x_prev=0, A_y_prev=0, A_z_prev=0, debug=False):
		accelCorrectionAmount = 0.03	# Percentage of Accelerometer Amount in Complementary Filter.
										# high: more accel noise, less responsivness of gyro included
										# low:  more gyro drift and slow correction, but high responsivness
		self.dt = dt					# sensor rate. needed for integration of gyros turn speed measurement
		self.debug = debug				# show more info
		
		### make (previous) Information available
		
		# from calcOrientation
		self.A_x_prev = A_x_prev
		self.A_y_prev = A_y_prev
		self.A_z_prev = A_z_prev
		
		# from angleFromGyro using previous calculations from calcOrientation
		self.A_x_gyro_prev = 0
		self.A_y_gyro_prev = 0
		self.A_z_gyro_prev = 0
		
		# [depreciated] from angleFromGyro_integrationOnly. with drift
		self.gyro_int_x = 0
		self.gyro_int_y = 0
		self.gyro_int_z = 0

		
	def calcOrientation(self, GyroData, AccelData):
		""" Combines accelerometer and gyroscope data to get robust Angle calculation. """

		# calc Angle with Gyroscope Data
		gyr_x, gyr_y, gyr_z = self.angleFromGyro(*GyroData)
		
		# calc Angle with Acceleration Data
			#if (9.81/2) < sqrt(AccelData[0]**2 + AccelData[1]**2 + AccelData[2]**2) < (9.81*2): # Acceleration seems valid
		acc_x, acc_y, acc_z = self.angleFromAccel(*AccelData)
			#print("## trust: {}<{}<{} = {}".format(9.81/2, sqrt(AccelData[0]**2 + AccelData[1]**2 + AccelData[2]**2), 9.81*2, (9.81/2) < sqrt(AccelData[0]**2 + AccelData[1]**2 + AccelData[2]**2) < (9.81*2)))
			#else:
			#print("## can't trust: {}<{}<{} = {}".format(9.81/2, sqrt(AccelData[0]**2 + AccelData[1]**2 + AccelData[2]**2), 9.81*2, (9.81/2) < sqrt(AccelData[0]**2 + AccelData[1]**2 + AccelData[2]**2) < (9.81*2)))
			# can't trust measurement, use prev calculation instead
			#acc_x, acc_y, acc_z = self.A_x_prev, self.A_y_prev, self.A_z_prev
		
		# Combine both calculated angles complementary
		# -> fast response from big gyro part
		# -> long term stability from a bit accel. nudges result to counter drift.
		self.A_x_prev = 0.98*gyr_x + 0.02*acc_x
		self.A_y_prev = 0.98*gyr_y + 0.02*acc_y
		self.A_z_prev = 0.98*gyr_z + 0.02*acc_z
		
		return self.A_x_prev, self.A_y_prev, self.A_z_prev


	def angleFromGyro(self, R_x, R_y, R_z):
		""" 
		Calc Angles A_x, A_y, A_z in Rad from improved Gyroscope Data. Uses integration (pass previous calculation and sense rate dt). Similar to end calculation except small Accel percentage.
		"""
		
		self.A_x_gyro_prev = R_x*self.dt + self.A_x_prev
		self.A_y_gyro_prev = R_y*self.dt + self.A_y_prev
		self.A_z_gyro_prev = R_z*self.dt + self.A_z_prev
		
		return self.A_x_gyro_prev, self.A_y_gyro_prev, self.A_z_gyro_prev
		

	def angleFromAccel(self, x, y, z):
		""" 
		Calc Angles A_x, A_y, A_z in Rad from raw Acceleration Data. Uses atan and multiple Axes for one angle. Returns Angles with High frequency noise aka jitter.
		"""
		
		# X-angle
		YZsquare = y**2 + z**2
		if x == 0:
			A_x = 0
		elif YZsquare == 0:
			A_x = pi/2
		else:
			A_x = atan( x / sqrt( y**2 + z**2 ) )
		
		# Y-angle
		XZsquare = x**2 + z**2
		if y == 0:
			A_y = 0
		elif XZsquare == 0:
			A_y = pi/2
		else:
			A_y = atan( y / sqrt( x**2 + z**2 ) )

		# Z-angle ??
		XYsquare = x**2 + y**2
		if z == 0:
			A_z = 0
		elif XYsquare == 0:
			A_z = pi/2
		else:
			A_z = atan( z / sqrt( x**2 + y**2 ) )

		return A_x, A_y, A_z


	def angleFromGyro_integrationOnly(self, R_x, R_y, R_z):
		""" [depreciated]
		Calc Angles A_x, A_y, A_z in Rad from raw Gyroscope Data. Uses integration (pass previous angles and sense rate dt). Returns Angles with Low frequency noise aka drift.
		"""
		
		self.gyro_int_x = R_x*self.dt + self.gyro_int_x
		self.gyro_int_y = R_y*self.dt + self.gyro_int_y
		self.gyro_int_z = R_z*self.dt + self.gyro_int_z
		
		return self.gyro_int_x, self.gyro_int_y, self.gyro_int_z



if __name__ == "__main__":
	Filter = ComplementaryFilter(debug=True)
	
	if len(sys.argv) >1:
		
		inInterpret = input("Arg is a File (1) or args are specific xyz values (2) ?")
		if "1" in inInterpret:
			with open(sys.argv[1], 'r') as f:
				print("opening file:", sys.argv[0])
				rawData = f.readlines()
				print("rawData", rawData)
			for line in rawData[1:]:
				line_list = line[:-2].replace(',', '.').split(';') # remove ;\n from end, change decimal seperator, split String into seperate measurements
				line_floats = [float(x) for x in line_list[1:]] # convert values to numbers
				line_dict = {'Timestamp': line_list[0], 'Accelerometer': line_floats[0:3], 'Gyroscope': line_floats[3:6]}
				print("\n" + str(line_dict))
				print("Orientation:\t(x: {:08.4f} |y: {:08.4f} |z: {:08.4f})".format(*Filter.calcOrientation(line_dict['Gyroscope'], line_dict['Accelerometer'])))
				print("\tAccel:\t(x: {:08.4f} |y: {:08.4f} |z: {:08.4f})".format(*Filter.angleFromAccel(*line_dict['Accelerometer'])))
				print("\tGyros:\t(x: {:08.4f} |y: {:08.4f} |z: {:08.4f})".format(*Filter.angleFromGyro_integrationOnly(*data_dict['Gyroscope'])))  # nicht doppelt berechnen
				###### data_dict????

				
		elif "2" in inInterpret:
			x, y, z = sys.argv[1:4]
			print("\nA_x: {}\nA_y: {}\nA_z: {}".format(*Filter.angleFromAccel(float(x), float(y), float(z))))
	
	else:
		print("printing help on class ComplementaryFilter:")
		print(ComplementaryFilter.__doc__)
		input("exiting... (press enter)")