import dijkstra
from collections import defaultdict
import cv2
import numpy as np
import collections

FILTER_THRESHOLD = 15000    # Walls filter threshold
PERSPECTIVE_WIDTH = 560		#Pixel Width
PERSPECTIVE_HEIGHT = 240	#Pixel Height
ROWS = 4				#The Number of rows in the physical maze
COLS = 7				#The number of columns in the physical maze
# PREVIOUS_SPHERO_COORD = [PERSPECTIVE_WIDTH//2, PERSPECTIVE_HEIGHT//2, 1]


class Maze_Solver():
    """ Maze Solver: uses camera info to provide checkpoints for controller.

        Maze solver uses inputs from the camera object to provide maze
        checkpoints for the Sphero controller to use in directing the Sphero to
        the end point.  In short, it uses filtered and raw maze images to locate
        walls, the endpoint, and the Sphero; with this information the solver
        then "solves" the maze using Dijkstra's algorithm. The results of the
        maze solving algorithms are checkpoints representing the shortest path
        the Sphero can use to get to the end point.
    """

    def __init__(self, camera, debug_mode=False):
        """ Load in camera object, setup up maze solving parameters/variables

        The camera object is loaded in to be used internally. Variables for
        past data (e.g. Sphero coordinates and maze layout) are established. The
        blob detector for the end point is created.  Debug mode flag is also
        loaded.

        :param camera: camera_main object.  Must be initialized first.
        :param debug_mode: Flag for debug mode
        """
        self.camera = camera
        self.previous_sphero_coords = [0,0]
        self.previous_mazes = collections.deque(maxlen=5)
        self.debug = debug_mode
        self.wall_img_debug = self.camera.get_image_wall_filtered(True).copy()
        self.end_detector = self.__create_end_detector()

    @staticmethod
    def __create_end_detector():
        """ Defines parameters for the endpoint blob detector

        :return: end point blob detector
        """
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

        return end_detector

    def getSpheroCorodinates(self):
        """ Finds the coordinates of the Sphero in the maze.

        Starts by collecting a transformed image (based on corners) and using a
        Houghes Circle transform to find the Sphero. If no/multiple Spheros are
        found, the previous coordinates will be returned instead.

        :return: Returns an array containing the [X, Y, Z] coordinates for the
                 Sphero.  These are pixel coordinates based on a transformed
                 adjusted image.
        """

        img = self.camera.get_image_unfiltered(transform=True)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if self.debug:
            cv2.imshow("Grayscale Maze", img)
            cv2.waitKey(5)

        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT,
                                   1.5, 75, param1=500, param2=30,
                                   minRadius=10, maxRadius=30)
        # No circles
        if circles is None or len(circles) == 0 or circles[0][0][0] == 0:
            return self.previous_sphero_coords
        # Multiple circles
        elif len(circles[0]) > 1:
            return self.previous_sphero_coords
        # One circle
        else:
            self.previous_sphero_coords = circles[0][0]
            return circles[0][0]

    def getStartPoint(self):
        """ Uses the current Sphero location as a starting point

        Converts the Sphero coordinates from pixel coordinates to maze
        coordinates for the maze matrix.

        :return: an array [X, Y] representing where the Sphero is currently
                located.
        """
        c = self.getSpheroCorodinates()
        return [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1,
                2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]

    def findEndMarker(self):
        """ Get coordinates of end markers within the maze.

        Starts by collecting a transformed image (based on corners) and using a
        Houghes Circle transform to find the endpoint. If no endpoint is found,
        the endpoint is reported as the upper-left corner (0, 0). If multiple
        endpoints are found, the first one is selected and returned

        :return:  a tuple [X, Y] containing pixel coordinates for the endpoint
        """
        endpoint_img = self.camera.get_image_endpoint_filtered(True)
        keypoints = self.end_detector.detect(endpoint_img)

        if len(keypoints)>1:
            print('Found multiple endpoints')
        if len(keypoints) == 0:
            print('No Endpoint found, routing to top left corner')
            return tuple([0, 0])
        print("findEndMarker:", keypoints[0].pt, type(keypoints[0].pt))
        return keypoints[0].pt

    def getEndPoint(self):
        """ Gets the maze matrix location of the endpoint

        Converts the pixel coordinates for the end point into the maze matrix
        location of the endpoint.

        :return: an array [X, Y] for the endpoint in the maze matrix
        """
        c = self.findEndMarker()
        return [2*int(c[1]*ROWS/PERSPECTIVE_HEIGHT)+1,
                2*int(c[0]*COLS/PERSPECTIVE_WIDTH)+1]

    def findMazeMatrix(self):
        """ Generates the maze matrix.

        A maze matrix is generated based on maze dimensions and the location of
        walls within the maze. Viable Sphero paths are represented with '1',
        invalid paths (e.g. wall) are denoted with a '0'.  A numpy matrix is
        initialed. Then the transformed maze image is mapped to the matrix.

        To find walls, a rectangle is drawn from the center of one maze cell to
        another. If the pixel value sum is greater than the threshold value,
        then it is assumed that there is a wall between these cells and that is
        marked in the maze matrix as a 0.

        :return: The median of the past 5 mazes as a numpy array.
        """
        # Get maze wall image
        walls_img = np.array(self.camera.get_image_wall_filtered(True))
        # Initialize maze matrix
        maze = np.zeros((2 * ROWS + 1, 2 * COLS + 1))
        maze[1:ROWS * 2:2, 1:COLS * 2:2] = 1
        # Maze debug image has lines drawn for viable paths
        self.wall_img_debug = walls_img.copy()

        col_width = int(PERSPECTIVE_WIDTH / COLS)
        row_height = int(PERSPECTIVE_HEIGHT / ROWS)

        for r in range(ROWS):
            for c in range(COLS):
                x_curr = int(col_width / 2 + c * col_width)
                y_curr = int(row_height / 2 + r * row_height)
                x_next = int(x_curr + col_width)
                y_next = int(y_curr + row_height)

                if c != COLS - 1:
                    #adds boxes to debug image for refrence and allignment checking (slightly shrunk to see individual boxes)
                    cv2.rectangle(self.wall_img_debug,
                                  (x_curr + col_width//4,
                                   y_curr - row_height//5),
                                  (x_next - col_width//4,
                                   y_curr + row_height//5),
                                  100)

                    #checks a rectangle for number of pixels, if above threshold it will assume there is a wall
                    temp = np.sum(walls_img[y_curr - row_height//4:y_curr + row_height//4,
                                  x_curr + col_width//4:x_next - col_width//4])
                    if temp < FILTER_THRESHOLD: #smaller dots should be less than FILTER_THRESHOLD and walls should be bigger
                        maze[r * 2 + 1, c * 2 + 2] = 1 #no wall here
                        cv2.line(self.wall_img_debug,(x_curr, y_curr), (x_next, y_curr), 200) #draws valid sphero paths on debug image

                #same as above except no comments
                if r != ROWS - 1:
                    cv2.rectangle(self.wall_img_debug,
                                  (x_curr + col_width//5,
                                   y_curr + row_height//4),
                                  (x_curr - col_width//5,
                                   y_next - row_height//4),
                                  100)

                    temp = np.sum(walls_img[y_curr + row_height//4:y_next - row_height//4,
                                  x_curr - col_width//4:x_curr + col_width//4])
                    if temp < FILTER_THRESHOLD:
                        maze[r * 2 + 2, c * 2 + 1] = 1
                        cv2.line(self.wall_img_debug,(x_curr, y_curr), (x_curr, y_next), 200)

        if(False): #debug stuff
            print('This is the maze:')
            print(maze)
            #cv2.imshow('Maze', self.wall_img_debug)
            #cv2.waitKey(5000)
        self.previous_mazes.append(maze)
        return np.median(self.previous_mazes, axis=0)  # Why the median?

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

        print("**positions: ", positions)
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
        # print("**Edges:", edges)
        try:
            checkpoints = dijkstra.shortestPath(edges, start, end)
        except:
            raise Exception('Dikstra Failed')

        #Now I pull out the checkpoints that are not corners
        del checkpoints[0]  # Delete initial point where Sphero located
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
    camera = Maze_Camera(True)
    solver = Maze_Solver(camera, True)
    #print(solver.getSpheroCorodinates())
    solver.findMazeMatrix()
    # start_pt = solver.getStartPoint()
    # start = (start_pt[0] - 1) * 5 + (start_pt[1] - 1) / 2
    # print("Start: " + str(start))
    # cv2.waitKey(10000)
    print(solver.solveMaze())


if __name__ == '__main__':
    main()
