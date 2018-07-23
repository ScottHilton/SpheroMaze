import dijkstra
from collections import defaultdict
import cv2
import numpy as np


PERSPECTIVE_WIDTH = 560		#Pixel Width
PERSPECTIVE_HEIGHT = 240	#Pixel Height
ROWS = 4				#The Number of rows in the physical maze
COLS = 7				#The number of columns in the physical maze
PREVIOUS_SPHERO_COORD = [PERSPECTIVE_WIDTH//2,PERSPECTIVE_HEIGHT//2,1]

# Parameters for the endpoint blob detector
params_end = cv2.SimpleBlobDetector_Params()
params_end.minDistBetweenBlobs = 10
params_end.filterByColor = True
params_end.blobColor = 255
params_end.filterByArea = True
params_end.minArea = 500
params_end.maxArea = 6000
params_end.filterByCircularity = False
params_end.filterByConvexity = False
params_end.filterByInertia = False
params_end.minInertiaRatio = 0.01
params_end.maxInertiaRatio = 1
end_detector = cv2.SimpleBlobDetector_create(params_end)


class Maze_Solver():
	def __init__(self, camera):
		self.camera = camera
		self.PREVIOUS_SPHERO_COORD = [0,0,0]

	def getSpheroCorodinates(self):
		img = self.camera.get_image_unfiltered(True)
		c = self.PREVIOUS_SPHERO_COORD

		GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		circles = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=90, param2=30, minRadius=10, maxRadius=30)

		circles = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 0.8, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		circles1 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		circles2 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.4, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		circles3 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		circles4 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.6, 75, param1=90, param2=30, minRadius=10, maxRadius=30)

		try:
			print("Circle: " + str(circles))
		except:
			print("No Circle")
		try:
			print("Circle1: " + str(circles1))
		except:
			print("No Circle1")
		try:
			print("Circle2: " + str(circles2))
		except:
			print("No Circle2")
		try:
			print("Circle3: " + str(circles3))
		except:
			print("No Circle3")
		try:
			print("Circle4: " + str(circles4))
		except:
			print("No Circle4")

		try:
			cv2.circle(GRAY, (int(circles[0]), int(circles[1])), 35, (255, 255, 255), 3)
			print('first')
		except:
			cv2.circle(GRAY, (int(c[0]), int(c[1])), 35, (255, 255, 255), 3)
			print('second')
			pass

		cv2.imshow('Sphero View', GRAY)


		if circles is None or len(circles)==0:
			print ('No Sphero Found on Image')
			return self.PREVIOUS_SPHERO_COORD
		else:
			if len(circles) > 1:
				print ('Found Multiple Circles')
			self.PREVIOUS_SPHERO_COORD = circles[0][0]
			return circles[0][0]

	def getStartPoint(self):
		c = self.getSpheroCorodinates()
		return [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1, 2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]

	def findEndMarker(self):
		endPoint_img = self.camera.get_image_endpoint_filtered(True)
		keypoints = end_detector.detect(endPoint_img)

		if len(keypoints)>1:
			print ('Found multiple endpoints')
		if len(keypoints) == 0:
			print ('No Endpoint found, routing to top left corner')
			# if debug == True:
			# 	cv2.imshow("Endpoint not found",endPoint_img)
			# 	cv2.waitKey(0)
			return (0,0)
		return keypoints[0].pt

	def getEndPoint(self):
		c = self.findEndMarker()
		return [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1, 2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]


	def findMazeMatrix(self):
		walls_img = self.camera.get_image_wall_filtered(True)
		sphero_coordinates = self.getSpheroCorodinates()

		r = int(sphero_coordinates[2] + 2)
		x = int(sphero_coordinates[0])
		y = int(sphero_coordinates[1])
		blank_image = np.zeros((2 * r+1, 2 * r+1), np.uint8)

		if (sphero_coordinates[2] != 0):
			walls_img[y - r:y + r, x - r:x + r] = 0#blank_image
			#print('hi')
			#print(walls_img[y - r:y + r, x - r:x + r])

		maze = np.zeros((2 * ROWS + 1, 2 * COLS + 1))
		maze[1:ROWS * 2:2, 1:COLS * 2:2] = 1

		wall_img_copy = walls_img.copy()

		for r in range(ROWS):
			for c in range(COLS):
				x_curr = int(PERSPECTIVE_WIDTH / COLS / 2 + c * PERSPECTIVE_WIDTH / COLS)
				y_curr = int(PERSPECTIVE_HEIGHT / ROWS / 2 + r * PERSPECTIVE_HEIGHT / ROWS)
				x_next = int(x_curr + PERSPECTIVE_WIDTH / COLS)
				y_next = int(y_curr + PERSPECTIVE_HEIGHT / ROWS)

				# print(x_curr,y_curr,type(x_curr),type(y_curr))
				cv2.line(wall_img_copy,(x_curr,y_curr),(x_curr,y_next),155)
				cv2.line(wall_img_copy,(x_curr, y_curr), (x_next, y_curr), 155)

				#print (walls_img[y_curr - 1:y_curr + 1, x_curr - 1:x_next + 1])
				temp = sum(sum(walls_img[y_curr - 1:y_curr + 1, x_curr - 1:x_next + 1]))
				#print(temp)
				#print(c)
				if temp / 255 < 3 and c != COLS - 1:
					maze[r * 2 + 1, c * 2 + 2] = 1
				temp = sum(sum(walls_img[y_curr - 1:y_next + 1, x_curr - 1:x_curr + 1]))

				if temp < 3 and r != ROWS - 1:
					maze[r * 2 + 2, c * 2 + 1] = 1
		print('This is the maze:')
		print(maze)
		#cv2.imshow('Maze', wall_img_copy)
		#cv2.waitKey(10)
		return maze

	def solveMaze(self):
		'''
		This code processes information for dijkstras formula then calls it to find the fastest path
		'''
		maze = self.findMazeMatrix()
		start_pt = self.getStartPoint()
		end_pt = self.getEndPoint()
		start = (start_pt[0] - 1) * 5 + (start_pt[1] - 1) / 2
		end = (end_pt[0] - 1) * 5 + (end_pt[1] - 1) / 2

		#positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
		positions = set() #following code populates the set automatically based on the rows and columns
		for row in range(0, ROWS):
			for col in range(0, COLS):
				positions.add(10 * row + col) # 10's position is rows 1's position is columns

		edges = defaultdict(dict) 					# Converts the 2 dimentional maze array to a dictionary of dictionaries
		for node in positions:						# to form a weighted graph for dijkstra to use
			if(node % 10 > 0):						# 10s position is rows 1s position is columns
				if (maze[(node // 10) * 2 + 1][(node % 10) * 2]): #there is no wall between node and node - 1
					edges[node][node - 1] = 1
					edges[node - 1][node] = 1
			if(node // 10 > 0):
				if (maze[(node // 10) * 2][(node % 10) * 2 + 1]): #there is no wall between node and node - 10
					edges[node][node - 10] = 1
					edges[node - 10][node] = 1

		#print("Edges: " + str(edges))
		print(" Start: " + str(start) + " End: " + str(end))
		try:
			checkpoints = dijkstra.shortestPath(edges, start, end)
		except:
			raise Exception('Dikstra Failed')
		#print ("Checkpoints: " + checkpoints)
		return checkpoints









######################################### DEBUGGING STUFF ############################################

	def test(self):
		maze = self.findMazeMatrix()
		start = 0
		end = 6
		#positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
		positions = set() #following code populates the set automatically based on the rows and columns
		for row in range(0, ROWS):
			for col in range(0, COLS):
				positions.add(10 * row + col) # 10's position is rows 1's position is columns

		edges = defaultdict(dict) 					# Converts the 2 dimentional maze array to a dictionary of dictionaries
		for node in positions:						# to form a weighted graph for dijkstra to use
			if(node % 10 > 0):						# 10s position is rows 1s position is columns
				if (maze[(node // 10) * 2 + 1][(node % 10) * 2]): #there is no wall between node and node - 1
					edges[node][node - 1] = 1
					edges[node - 1][node] = 1
			if(node // 10 > 0):
				if (maze[(node // 10) * 2][(node % 10) * 2 + 1]): #there is no wall between node and node - 10
					edges[node][node - 10] = 1
					edges[node - 10][node] = 1

		#print("Edges: " + str(edges))
		print(" Start: " + str(start) + " End: " + str(end))
		checkpoints = dijkstra.shortestPath(edges, start, end)
		#print ("Checkpoints: " + checkpoints)
		return checkpoints






def main():
	from camera_main import Maze_Camera
	camera = Maze_Camera()
	solver = Maze_Solver(camera)
	print(solver.getSpheroCorodinates())
	#solver.findMazeMatrix()
	#start_pt = solver.getStartPoint()
	#start = (start_pt[0] - 1) * 5 + (start_pt[1] - 1) / 2
	#print("Start: " + str(start))
	#cv2.waitKey(10000)
	#print(solver.test())

if __name__ == '__main__':
    main()






		#circles = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 0.8, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		# circles = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		# circles2 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.4, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		# circles3 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=90, param2=30, minRadius=10, maxRadius=30)
		# circles4 = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.6, 75, param1=90, param2=30, minRadius=10, maxRadius=30)

		# try:
		# 	print("Circle: " + str(circles))
		# except:
		# 	print("No Circle")
		# try:
		# 	print("Circle1: " + str(circles1))
		# except:
		# 	print("No Circle1")
		# try:
		# 	print("Circle2: " + str(circles2))
		# except:
		# 	print("No Circle2")
		# try:
		# 	print("Circle3: " + str(circles3))
		# except:
		# 	print("No Circle3")
		# try:
		# 	print("Circle4: " + str(circles4))
		# except:
		# 	print("No Circle4")

		# try:
		# 	cv2.circle(GRAY, (int(circles[0]), int(circles[1])), 35, (255, 255, 255), 3)
		# 	print('first')
		# except:
		# 	cv2.circle(GRAY, (int(c[0]), int(c[1])), 35, (255, 255, 255), 3)
		# 	print('second')
		# 	pass
		#
		# cv2.imshow('Sphero View', GRAY)
