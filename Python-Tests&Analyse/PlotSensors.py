import sys, traceback
import numpy as np
import multiprocessing
import mpl_toolkits.mplot3d.axes3d as p3  # This import registers the 3D projection, but is otherwise unused.
import matplotlib.gridspec as gridspec  # for subplot organisation
from matplotlib import pyplot as plt
from matplotlib import animation
from math import sin, cos, pi
from TiltFilter import ComplementaryFilter


class Cube:
	def __init__(self, ax, intervals=[1, 1, 1]):
		self.intervals = intervals
		dx = self.intervals[0]
		dy = self.intervals[1]
		dz = self.intervals[2]
		self.points = [(dx,dy,-dz), (-dx,dy,-dz), (-dx,-dy,-dz), (dx,-dy,-dz), (dx,dy,dz), (-dx,dy,dz), (-dx,-dy,dz), (dx,-dy,dz)]
		self.edges  = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)] # numbers correspond to indeces of Vertices in self.points
		self.internalXAxis = (2,3) # numbers correspond to indeces of Vertices in self.points
		self.internalYAxis = (2,1)
		self.internalZAxis = (2,6)
		self.edgeCount = 0
		
		# make line plots as cube
		self.lines = []
		for s, e in self.edges: # All Edges (numbered Vertices)
			start = self.points[s]
			end   = self.points[e]
			self.lines.append(ax.plot([start[0], end[0]], [start[1], end[1]])[0])	# make one line
			self.lines[-1].set_3d_properties([start[2], end[2]])					# add z compnent
		
	def drawCube(self):
		# Draw lines
		self.edgeCount = 0
		for s, e in self.edges: # All Edges (numbered Vertices)
			start = self.points[s]
			end   = self.points[e]
			self.lines[self.edgeCount].set_data([start[0], end[0]], [start[1], end[1]]) # move one line
			self.lines[self.edgeCount].set_3d_properties([start[2], end[2]])
			self.edgeCount += 1 	# inc counter for next line

	def rotateCube(self, AngleX, AngleY, AngleZ):
		#print("TypesAngles:", type(AngleX), type(AngleY), type(angleZ))
		#print(AngleX, AngleY, angleZ)
		# reset Points for absolut angles
		dx = self.intervals[0]
		dy = self.intervals[1]
		dz = self.intervals[2]
		self.points = [(dx,dy,-dz), (-dx,dy,-dz), (-dx,-dy,-dz), (dx,-dy,-dz), (dx,dy,dz), (-dx,dy,dz), (-dx,-dy,dz), (dx,-dy,dz)]
		try:

			# rotate all points around all 3 axes:
			# 	rotate Obj around Z-Axis
			self.rotateZ(AngleZ)
			
			# 	rotate Obj around Y-Axis
			#		calc angles .............
			vecYintern = np.array(self.points[self.internalYAxis[1]]) - np.array(self.points[self.internalYAxis[0]])
			#print("vecYintern:", vecYintern)
			
									# 1. rotate Obj so that (intern Y-Axis)=(global Z-Axis)
			rotX = self.vectorAngle(vecYintern, np.array([vecYintern[0], 0, vecYintern[2]]))
			#print("rotX", rotX)
			if rotX is not None:
				self.rotateX(rotX)
			
			rotY = self.vectorAngle(vecYintern, np.array([0, vecYintern[1], vecYintern[2]]))
			#print("rotY", rotY)
			if rotY is not None:
				self.rotateY(rotY)
			
			self.rotateZ(AngleY)	# 2. rotate Obj around (intern Y-Axis)=(global Z-Axis)
			if rotY is not None:
				self.rotateY(-rotY)		# 3. reverse rotation in 1.
			if rotX is not None:
				self.rotateX(-rotX)
			
			# 	rotate Obj around X-Axis
			vecXintern = np.array(self.points[self.internalXAxis[1]]) - np.array(self.points[self.internalXAxis[0]])
			#print("vecXintern:", vecXintern)
			
									# 1. rotate Obj so that (intern Y-Axis)=(global Z-Axis)
			rotX = self.vectorAngle(vecXintern, np.array([vecXintern[0], 0, vecXintern[2]]))
			#print("rotX", rotX)
			if rotX is not None:
				self.rotateX(rotX)
			
			rotY = self.vectorAngle(vecXintern, np.array([0, vecXintern[1], vecXintern[2]]))
			#print("rotY", rotY)
			if rotY is not None:
				self.rotateY(rotY)
			
			self.rotateZ(AngleX)	# 2. rotate Obj around (intern Y-Axis)=(global Z-Axis)
			if rotY is not None:
				self.rotateY(-rotY)		# 3. reverse rotation in 1.
			if rotX is not None:
				self.rotateX(-rotX)
			
			self.drawCube()
		except Exception as err:
				traceback.print_exc(file=sys.stdout)
		
	def rotateX(self, angle):
		cosA = cos(angle)
		sinA = sin(angle)
		
		rotated = []
		for p in self.points:
			rotated.append([p[0],						# x
							p[1] * cosA - p[2] * sinA,  # y
							p[1] * sinA + p[2] * cosA]) # z
		self.points = rotated
	
	def rotateY(self, angle):
		cosA = cos(angle)
		sinA = sin(angle)
		
		rotated = []
		for p in self.points:
			rotated.append([p[0] * cosA - p[2] * sinA,  # x
							p[1],						# y
							p[0] * sinA + p[2] * cosA]) # z
		self.points = rotated
	
	def rotateZ(self, angle):
		cosA = cos(angle)
		sinA = sin(angle)
		
		rotated = []
		for p in self.points:
			rotated.append([p[0] * cosA - p[1] * sinA,  # x
							p[0] * sinA + p[1] * cosA,  # y
							p[2]])						# z
		self.points = rotated

	def vectorAngle(self, v1, v2):
		""" Returns the angle in radians between vectors v1, v2 """
		# norm
		lengthV1 = np.linalg.norm(v1)
		lengthV2 = np.linalg.norm(v2)
		if lengthV1 != 0 and lengthV2 != 0:
			v1_norm = v1 / np.linalg.norm(v1)
			v2_norm = v2 / np.linalg.norm(v2)
			
			return np.arccos(np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0))



class PlotRunning:
	def __init__(self):
	
		self.errorCount = 0
		
		#ax = fig.gca(projection='3d')
		#fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))
		self.fig = plt.figure()
		ax = self.fig.add_subplot(111, projection='3d') # 111 = nrows, ncols, index = 1, 1, 1

		# Setting the axes properties
		ax.set_xlim3d([-10, 10])
		ax.set_xlabel('X')

		ax.set_ylim3d([-10, 10])
		ax.set_ylabel('Y')

		ax.set_zlim3d([-10, 10])
		ax.set_zlabel('Z')
		
		
		#global quiver
		#quiver = ax.quiver([0,0],[0,0],[0,0],[1,1],[1,1],[1,1])	# X, Y, Z, dX, dV, dW ###, [(1,0,0),(0,0,1)]
		#quiver = ax.quiver(*get_arrow(Data[0]))
		self.lines = [ax.plot([0, 1], [0,0], [0,0])[0] for i in range(2)] #[ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]
		self.cube = Cube(ax, intervals=[0.5,1.2,2])
		
		#global quiver2
		#quiver2 = ax.quiver(*get_arrow(Data[0]))


	def startUpdating(self, Data, sampleInterval=100):
		ani = animation.FuncAnimation(self.fig, self.update, fargs=None, \
										frames=Data, interval=sampleInterval, \
										repeat_delay=3000)
		plt.show()

	def update(self, frame, DataFrame=None):
		nextReading = None
		
		if type(frame) == dict:
			nextReading = frame
		elif type(DataFrame) == multiprocessing.connection.PipeConnection:
			if DataFrame.poll():  # check if data available
				while DataFrame.poll():
					nextReading = DataFrame.recv()
				print("read", nextReading)
			else:
				self.errorCount += 1
				if self.errorCount > 10:
					self.errorCount = 1
					print("Catch up!")
			
		else:
			print("type(nextReading):", type(DataFrame))
		
		centerX = 0					# Center X-Position
		centerY = 0					# Center Y-Position
		centerZ = 0					# Center Z-Position
		linecount = 0
		try:
			x,y,z = nextReading['Accelerometer']
			self.lines[linecount].set_data([centerX, x], [centerY, y])
			self.lines[linecount].set_3d_properties([centerZ, z])
			linecount += 1
		except:
			pass
		try:
			x,y,z = nextReading['Gyroscope']
			self.lines[linecount].set_data([centerX, x], [centerY, y])
			self.lines[linecount].set_3d_properties([centerZ, z])
		except:
			pass
		try:
			Ax,Ay,Az = nextReading['Calculated Angle']
			self.cube.rotateCube(Ax,Ay,Az)
		except:
			pass



class PlotLog:
	def __init__(self, dt=100):
		self.errorCount = 0
	
		self.pastData = {'Calculated Angle': {'x':[], 'y':[], 'z':[]}, 'Accelerometer': {'x':[], 'y':[], 'z':[]}, 'Gyroscope': {'x':[], 'y':[], 'z':[]}}
		self.xTime = list(np.arange(-50*dt, 0, dt))
		self.dt = dt
	
		self.fig = plt.figure()
		grid = gridspec.GridSpec(ncols=1, nrows=3)
		self.ax1 = self.fig.add_subplot(grid[0,0]) #111)
		self.ax2 = self.fig.add_subplot(grid[1,0]) #212)
		self.ax3 = self.fig.add_subplot(grid[2,0]) #313)
		
		self.ax1.set_title('Accelerometer')
		self.ax2.set_title('Gyroscope')
		self.ax3.set_title('Calculated Angle')
		# Format plot
		#plt.xticks(rotation=45, ha='right')
		#plt.subplots_adjust(bottom=0.30)
		plt.title('Sensors')
		#plt.ylabel('Temperature (deg C)')
		
	def startUpdating(self, Data, sampleInterval=100):
		ani = animation.FuncAnimation(self.fig, self.update, fargs=None, \
										frames=Data, interval=sampleInterval, \
										repeat_delay=10000)
		plt.show()
		
	def update(self, frame, DataFrame=None):
		nextReading = None
		
		if type(frame) == dict:
			nextReading = frame
		elif type(DataFrame) == multiprocessing.connection.PipeConnection:
			if DataFrame.poll():  # check if data available
				while DataFrame.poll():
					nextReading = DataFrame.recv()
				print("read", nextReading)
			else:
				self.errorCount += 1
				if self.errorCount > 10:
					self.errorCount = 1
					print("Catch up!")
			
		else:
			print("type(nextReading):", type(DataFrame))
			return
		
		try:
			# Add new and cut out too old data
			self.pastData['Accelerometer']['x'].append(nextReading['Accelerometer'][0])
			self.pastData['Accelerometer']['y'].append(nextReading['Accelerometer'][1])
			self.pastData['Accelerometer']['z'].append(nextReading['Accelerometer'][2])
			if len(self.pastData['Accelerometer']['x']) > 50:
				self.pastData['Accelerometer']['x'] = self.pastData['Accelerometer']['x'][-50:]
				self.pastData['Accelerometer']['y'] = self.pastData['Accelerometer']['y'][-50:]
				self.pastData['Accelerometer']['z'] = self.pastData['Accelerometer']['z'][-50:]
			
			# Draw x and y lists
			self.ax1.clear()
			self.ax1.plot(np.arange(-len(self.pastData['Accelerometer']['x'])*self.dt, 0, self.dt), self.pastData['Accelerometer']['x'])
			self.ax1.plot(np.arange(-len(self.pastData['Accelerometer']['y'])*self.dt, 0, self.dt), self.pastData['Accelerometer']['y'])
			self.ax1.plot(np.arange(-len(self.pastData['Accelerometer']['z'])*self.dt, 0, self.dt), self.pastData['Accelerometer']['z'])
		except Exception as err:
			traceback.print_exc(file=sys.stdout)

		try:
			# Add new and cut out too old data
			self.pastData['Gyroscope']['x'].append(nextReading['Gyroscope'][0])
			self.pastData['Gyroscope']['y'].append(nextReading['Gyroscope'][1])
			self.pastData['Gyroscope']['z'].append(nextReading['Gyroscope'][2])
			if len(self.pastData['Gyroscope']['x']) > 50:
				self.pastData['Gyroscope']['x'] = self.pastData['Gyroscope']['x'][-50:]
				self.pastData['Gyroscope']['y'] = self.pastData['Gyroscope']['y'][-50:]
				self.pastData['Gyroscope']['z'] = self.pastData['Gyroscope']['z'][-50:]
			
			# Draw x and y lists
			self.ax2.clear()
			self.ax2.plot(np.arange(-len(self.pastData['Gyroscope']['x'])*self.dt, 0, self.dt), self.pastData['Gyroscope']['x'])
			self.ax2.plot(np.arange(-len(self.pastData['Gyroscope']['y'])*self.dt, 0, self.dt), self.pastData['Gyroscope']['y'])
			self.ax2.plot(np.arange(-len(self.pastData['Gyroscope']['z'])*self.dt, 0, self.dt), self.pastData['Gyroscope']['z'])
		except Exception as err:
			traceback.print_exc(file=sys.stdout)

		try:
			# Add new and cut out too old data
			self.pastData['Calculated Angle']['x'].append(nextReading['Calculated Angle'][0])
			self.pastData['Calculated Angle']['y'].append(nextReading['Calculated Angle'][1])
			self.pastData['Calculated Angle']['z'].append(nextReading['Calculated Angle'][2])
			print("Z Orientation:", nextReading['Calculated Angle'][2])
			if len(self.pastData['Calculated Angle']['x']) > 50:
				self.pastData['Calculated Angle']['x'] = self.pastData['Calculated Angle']['x'][-50:]
				self.pastData['Calculated Angle']['y'] = self.pastData['Calculated Angle']['y'][-50:]
				self.pastData['Calculated Angle']['z'] = self.pastData['Calculated Angle']['z'][-50:]

			# Draw x and y lists
			self.ax3.clear()
			self.ax3.plot(np.arange(-len(self.pastData['Calculated Angle']['x'])*self.dt, 0, self.dt), self.pastData['Calculated Angle']['x'])
			self.ax3.plot(np.arange(-len(self.pastData['Calculated Angle']['y'])*self.dt, 0, self.dt), self.pastData['Calculated Angle']['y'])
			self.ax3.plot(np.arange(-len(self.pastData['Calculated Angle']['z'])*self.dt, 0, self.dt), self.pastData['Calculated Angle']['z'])
		except Exception as err:
			pass#traceback.print_exc(file=sys.stdout)
			
		self.ax1.set_title('Accelerometer')
		self.ax2.set_title('Gyroscope')
		self.ax3.set_title('Calculated Angle')
	
#testing
# showtime = PlotRunning()
# showtime.startUpdating([{'Calculated Angle': [0,0,0]},\
						# {'Calculated Angle': [0.1,0,0]},\
						# {'Calculated Angle': [0.2,0,0]},\
						# {'Calculated Angle': [0.3,0,0]},\
						# {'Calculated Angle': [0.4,0,0]},\
						# {'Calculated Angle': [0.5,0,0]},\
						# {'Calculated Angle': [0.5,0,0]},\
						# {'Calculated Angle': [0.5,0,0]},\
						# {'Calculated Angle': [0.5,0,0]},\
						# {'Calculated Angle': [0.5,0,0]},\
						# ], sampleInterval=500)
if __name__ == "__main__":

	# Kommandozeilen Argumente
	DataFiles = [] # Liste der Dateien
	if len(sys.argv) > 1:
		print( "{} Dateien:\n{}".format(len(sys.argv)-1, '\n'.join(sys.argv[1:])) )
		DataFiles = sys.argv[1:]
	else:
		DataFiles = input("Input CSV-File with Sensor Data:").split(';')
	
	# Winkelberechnung
	Filter = ComplementaryFilter()
	
	if DataFiles:
		with open(DataFiles[0], 'r') as f:
			rawData = f.readlines()
		
		keys = rawData[0].split(';')
		Data = []
		for line in rawData[1:]:
			line_list = line[:-2].replace(',', '.').split(';') # remove ;\n from end, change decimal seperator, split String into seperate measurements
			line_floats = [float(x) for x in line_list[1:]] # convert values to numbers
			#line_floats.insert(0, line_list[0]) # add timestamp back to float values
			line_dict = {'Timestamp': line_list[0], 'Accelerometer': line_floats[0:3], 'Gyroscope': line_floats[3:6], 'Calculated Angle': Filter.calcOrientation(line_floats[3:6], line_floats[0:3])}
			print(line_dict)
			Data.append(line_dict)
		#print(Data)
	else:
		print("NO DATA")
		sys.exit()
	
	showtime1 = PlotLog(0.1) #dt=Abtastrate/Samplerate
	showtime2 = PlotRunning()
	showtime1.startUpdating(Data)
	showtime2.startUpdating(Data)