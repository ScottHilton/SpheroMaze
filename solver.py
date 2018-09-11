import dijkstra
from collections import defaultdict
import cv2
import numpy as np
import collections

FILTER_THRESHOLD = 15000
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
		self.previous_sphero_coords = [0,0]
		self.previous_mazes = collections.deque(maxlen = 5)

	def getSpheroCorodinates(self):
		img = self.camera.get_image_unfiltered(True)

		GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		circles = cv2.HoughCircles(GRAY, cv2.HOUGH_GRADIENT, 1.5, 75, param1=500, param2=30, minRadius=10, maxRadius=30)

		if circles is None or len(circles)== 0 or circles[0][0][0] == 0 :
			#print ('No Sphero Found on Image')
			return self.previous_sphero_coords
		else:
			if len(circles[0]) > 1:
				#print ('Found Multiple Circles: ' + str(circles))
				return self.previous_sphero_coords
			self.previous_sphero_coords = circles[0][0]
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
			return (0,0)
		return keypoints[0].pt

	def getEndPoint(self):
		c = self.findEndMarker()
		return [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1, 2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]


	def findMazeMatrix(self):
		walls_img = np.array(self.camera.get_image_wall_filtered(True))
		sphero_coordinates = self.getSpheroCorodinates()

		maze = np.zeros((2 * ROWS + 1, 2 * COLS + 1))
		maze[1:ROWS * 2:2, 1:COLS * 2:2] = 1

		self.wall_img_debug = walls_img.copy()

		colW = int(PERSPECTIVE_WIDTH / COLS)
		rowH = int(PERSPECTIVE_HEIGHT / ROWS)

		for r in range(ROWS):
			for c in range(COLS):
				x_curr = int(colW / 2 + c * colW)
				y_curr = int(rowH / 2 + r * rowH)
				x_next = int(x_curr + colW)
				y_next = int(y_curr + rowH)

				if c != COLS - 1:
					#adds boxes to debug image for refrence and allignment checking (slightly shrunk to see individual boxes)
					#cv2.rectangle(self.wall_img_debug,(x_curr + colW//4 , y_curr - rowH//5) , (x_next - colW//4 , y_curr + rowH//5),100)

					#checks a rectangle for number of pixels, if above threshold it will assume there is a wall
					temp = np.sum(walls_img[y_curr - rowH//4:y_curr + rowH//4, x_curr + colW//4:x_next - colW//4])
					if temp < FILTER_THRESHOLD: #smaller dots should be less than FILTER_THRESHOLD and walls should be bigger
						maze[r * 2 + 1, c * 2 + 2] = 1 #no wall here
						cv2.line(self.wall_img_debug,(x_curr, y_curr), (x_next, y_curr), 200) #draws valid sphero paths on debug image

				#same as above except no comments
				if r != ROWS - 1:
					#cv2.rectangle(self.wall_img_debug,(x_curr + colW//5 , y_curr + rowH//4) , (x_curr - colW//5 , y_next - rowH//4),100)
					temp = np.sum(walls_img[y_curr + rowH//4:y_next - rowH//4, x_curr - colW//4:x_curr + colW//4])
					if temp < FILTER_THRESHOLD:
						maze[r * 2 + 2, c * 2 + 1] = 1
						cv2.line(self.wall_img_debug,(x_curr, y_curr), (x_curr, y_next), 200)

		if(False): #debug stuff
			print('This is the maze:')
			print(maze)
			#cv2.imshow('Maze', self.wall_img_debug)
			#cv2.waitKey(5000)
		self.previous_mazes.append(maze)
		return np.median(self.previous_mazes, axis = 0)

	def coord_to_dik_num(self, c):
		array_pos = [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1, 2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]
		return (array_pos[0] - 1) * 5 + (array_pos[1] - 1) / 2

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

		try:
			checkpoints = dijkstra.shortestPath(edges, start, end)
		except:
			raise Exception('Dikstra Failed')

		#Now I pull out the checkpoints that are not corners
		del checkpoints[0]
		i = len(checkpoints) - 2
		while i > 0:
			if checkpoints[i] % 10 == checkpoints[i + 1] % 10 and checkpoints[i] % 10 == checkpoints[i - 1] % 10:
				del checkpoints[i]
			elif checkpoints[i] // 10 == checkpoints[i + 1] // 10 and checkpoints[i] // 10 == checkpoints[i - 1] // 10:
				del checkpoints[i]
			i = i - 1

		return checkpoints


#run main only for debuging

def main():
	from camera_main import Maze_Camera
	camera = Maze_Camera()
	solver = Maze_Solver(camera)
	#print(solver.getSpheroCorodinates())
	solver.findMazeMatrix()
	# start_pt = solver.getStartPoint()
	# start = (start_pt[0] - 1) * 5 + (start_pt[1] - 1) / 2
	# print("Start: " + str(start))
	# cv2.waitKey(10000)
	# print(solver.solveMaze())

if __name__ == '__main__':
    main()
