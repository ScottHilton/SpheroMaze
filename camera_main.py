#####################################################################
# Brigham Young University
# Sphero Maze Runner
# ECEn Department Demo
# Camera Main Code
#
#####################################################################

# Written in Python 3
# Version 0.3 (Prototype)
# About this version
# August 6, 2018
# 1.  Added OS independent support (unique features for different OS platforms)
# 2.  Modified camera parameters for OS independence

# Imports
import cv2
import time
import numpy as np
import json
import os

# Support Macros
CAMERA_NUMBER = 0  # The camera number indicates which camera is being used; default value is 0.
PARAMETERS_FILE = "parameters.txt"  # Name of the file that stores threshold values
CORNERS_FILE = "corners.txt"
CAMERA_SETTINGS_FILE = "camSettings.txt"
CAM_MAX_EXPOSURE = 20000  # Maximum exposure value
CAM_MAX_BRIGHTNESS = 100.0  # Maximum brightness value
CAM_INITIAL_BRIGHTNESS = 100  # The brightness will be set to this value upon initialization
CAM_INITIAL_EXPOSURE = 10  # The exposure will be set to this value upon initialization

# Linux Camera Parameters
CAM_MAX_EXPOSURE_LINUX = 200  # Maximum exposure value
CAM_MAX_BRIGHTNESS_LINUX = 100.0  # Maximum brightness value
CAM_MIN_EXPOSURE_LINUX = 0  # Minimum exposure value
CAM_MIN_BRIGHTNESS_LINUX = 0  # Minimum brightness value
CAM_INITIAL_BRIGHTNESS_LINUX = 100  # The brightness will be set to this value upon initialization
CAM_INITIAL_EXPOSURE_LINUX = 50  # The exposure will be set to this value upon initialization
# Windows Camera Parameters
CAM_MAX_EXPOSURE_WIN = 0  # Maximum exposure value
CAM_MAX_BRIGHTNESS_WIN = 50  # Maximum brightness value
CAM_MIN_EXPOSURE_WIN = -13  # Minimum exposure value
CAM_MIN_BRIGHTNESS_WIN = 0  # Minimum brightness value
CAM_INITIAL_BRIGHTNESS_WIN = 25  # The brightness will be set to this value upon initialization
CAM_INITIAL_EXPOSURE_WIN = -10  # The exposure will be set to this value upon initialization

PERSPECTIVE_WIDTH = 560		#Pixel Width
PERSPECTIVE_HEIGHT = 240	#Pixel Height
kernel = np.ones((3,3),np.uint8)  #### What does this do?  ####
NOCAM_IMG = 'maze3.jpg'

#####################################################################
# The purpose of this code is to provide an interface for the maze
# code to interact with the camera. This code will handle the camera
# object, camera settings, and filter settings. This code will also
# handle functionality for corner coordinates.
#####################################################################

# This class will handle the camera and the filters
class Maze_Camera():
    def __init__(self, nocam = False):
        self.noCam = nocam # Flag for no camera debug mode

        # Camera Parameters (Initialized to LINUX values, later changed by check_OS function)
        self.cam_brightness_init = CAM_INITIAL_BRIGHTNESS  # Holds current brightness value
        self.cam_exposure_init = CAM_INITIAL_EXPOSURE  # Holds current exposure value
        self.cam_brightness_min = CAM_MIN_BRIGHTNESS_LINUX  # Holds min brightness value
        self.cam_brightness_max = CAM_MAX_BRIGHTNESS_LINUX  # Holds max brightness value
        self.cam_exposure_min = CAM_MIN_EXPOSURE_LINUX  # Holds min exposure value
        self.cam_exposure_max = CAM_MAX_EXPOSURE_LINUX  # Holds max exposure value
        self.OS = self.__check_os()  # Determine the OS and change camera parameters as needed

        # CAMERA
        self.camera_open = False  # Flag is true if the camera is open
        self.camera_setup = False # Flag is true if the camera settings have been configured (brightness, exposure, etc)
        if not self.noCam:
            self.cap = self.__open_camera()  # Open and collect camera object
            self.__setup_camera()  # Set up the camera settings

        # FILTERS
        # Initialize the thresholds for walls, corners, and endPoints
        self.wallsThreshold = self.Threshold("Walls Threshold")
        # self.cornersThreshold = self.Threshold("Corners Threshold")  # Unneeded
        self.endPointThreshold = self.Threshold("Endpoint Threshold")
        # print("Test:",self.wallsThreshold.name, self.cornersThreshold.name, self.endPointThreshold.name)
        self.spheroThreshold = self.Threshold("sphero Threshold")
        # print("Test:",self.wallsThreshold.name, self.cornersThreshold.name, self.spheroThreshold.name)
        self.__init_thresholds()  # Set values for thresholds

        # Corners
        #self.corners = [] # This will hold the coordinates of each corner
        #self.corners_set = False  # Flag is true if corners have been set
        #self.maze_ROI = {'row1': 0}  # Maze Region of Interest (ROI)
        self.__load_corners() #Loads Corners from file

    def __del__(self):
        # Try to close the camera
        try:
            if self.cap:
                self.cap.release()
            print("Maze Camera: Closing Camera")
        except:
            print("Maze Camera: Hard Close")

        # Reset flags
        self.camera_open = False
        self.camera_setup = False

# ------------------------------------- Threshold Class ---------------------------------------------------------------#
    # The HSV threshold class used for storing filter thresholds values for each filter
    class Threshold():
        def __init__(self,threshold_name):
            self.name = threshold_name  # Identifying name
        # HSV values
        H_MIN = 0
        H_MAX = 256
        S_MIN = 0
        S_MAX = 256
        V_MIN = 0
        V_MAX = 256

# ------------------------------------- Camera Setup ------------------------------------------------------------------#
    # Open camera will open up and return a camera object(video capture).
    def __open_camera(self):
        # Open the camera
        try:
            camera = cv2.VideoCapture(CAMERA_NUMBER)
        # Sound the alarm if the camera fails to open
        except Exception as error:
            print("Maze Camera: Init Error: Failed to open the camera for the following reason:")
            print(error)

        # Check if the opened
        if camera.isOpened():
            self.camera_open = True
            print("Maze Camera Opened")
            return camera
        else:
            print("Maze Camera: Init Error: Failed to open camera")
            return

    # Setup camera will initialize the settings for the camera
    def __setup_camera(self):
        # Initialize camera brightness and exposure
        self.cam_brightness_value = self.cam_brightness_init
        self.cam_exposure_value = self.cam_exposure_init
        # Apply exposure and brightness values to camera
        if self.camera_open:
            try:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # This is needed in Linux to allow changes in exposure
                # Windows
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.cam_brightness_value)  # Set Brightness
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, self.cam_exposure_value)  # set Exposure
                # Linux
                else:
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS,
                                 (self.cam_brightness_value / CAM_MAX_BRIGHTNESS_LINUX))  # Set Brightness
                    self.cap.set(cv2.CAP_PROP_EXPOSURE,
                                 (self.cam_exposure_value / CAM_MAX_EXPOSURE_LINUX))  # set Exposure
                self.__load_cam_settings()
                self.camera_setup = True  # Set flag
            except:
                print("Maze Camera Error:  Failed to setup camera")

# -------------------- Reset functions for camera and filters ----------------------------------------------------------#
    # The reset camera function will close the camera, and then reopen the camera
    def reset_camera(self):
        try:
            self.cap.release()  # Release the camera
            self.__open_camera()  # Open camera again
            self.__setup_camera()  # Setup camera again
            time.sleep(0.1)  # Pause for a moment
            print("Maze Camera Reset")
        except:
            print("Maze Camera failed to reset")

# ------------------------------ Set Brightness and Exposure ----------------------------------------------------------#
    # Set camera exposure
    def set_exposure(self, x):
        if self.camera_open and self.camera_setup:
            try:
                self.cam_exposure_value = int(x)
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, int(x))
                else:
                    self.cap.set(cv2.CAP_PROP_EXPOSURE , int(x) / CAM_MAX_EXPOSURE_LINUX)
            except:
                print("Maze Camera: set_exposure error")
        else:
            print("Maze Camera: set_exposure error: camera not ready")

    # Set camera brightness
    def set_brightness(self, x):
        if self.camera_open and self.camera_setup:
            try:
                self.cam_brightness_value = int(x)
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS, int(x))
                else:
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS , int(x) / CAM_MAX_BRIGHTNESS_LINUX)
            except:
                print("Maze Camera: set_brightness error")
        else:
            print("Maze Camera: set_brightness error: camera not ready")

# ------------------------------ Save Brightness and Exposure ---------------------------------------------------------#
    # This function will write the current camera settings to a text file
    def save_cam_settings(self):
        print("Maze Camera: Saving camera settings to file")
        with open(CAMERA_SETTINGS_FILE, "w") as f:
            json.dump((self.cam_brightness_value, self.cam_exposure_value), f)



    # This function will read in camera settings from a text file
    def __load_cam_settings(self):
        try:
            with open(CAMERA_SETTINGS_FILE) as f:
                self.cam_brightness_value, self.cam_exposure_value = json.load(f)
            print("Maze Camera: Loading previous brightness and exposure settings from file: ", self.cam_brightness_value, self.cam_exposure_value)
            #print((self.cam_brightness_value, self.cam_exposure_value))
        except:
            print("Maze Camera: Unable to load brightness and exposure settings from camSettings.txt.")

# -------------------- Filters and Stuff ----------------------------------------------------------#
    # This function will read in threshold values from a text file
    def __init_thresholds(self):
        # (Copy and pasted from maze_runner_v2.py)
        print("Maze Camera: Loading filter values from param file")
        f = open(PARAMETERS_FILE, 'r')

        self.__read_values_from_file(f, self.wallsThreshold)
        self.__read_values_from_file(f, self.endPointThreshold)
        self.__read_values_from_file(f, self.spheroThreshold)

        f.close()

    # This function will write the current threshold values to a text file
    def save_threshold_values(self):
        print("Maze Camera: Saving filter values to param file")
        f = open(PARAMETERS_FILE, 'w')
        self.__write_values_to_file(f, self.wallsThreshold)
        self.__write_values_to_file(f, self.endPointThreshold)
        self.__write_values_to_file(f, self.spheroThreshold)
        f.close()

    # Supporting function that reads values into a threshold class.
    # Args: file is the file object to read from; threshold_class
    def __read_values_from_file(self, file, threshold_class):
        try:
            threshold_class.H_MIN = int(file.readline())
            threshold_class.H_MAX = int(file.readline())
            threshold_class.S_MIN = int(file.readline())
            threshold_class.S_MAX = int(file.readline())
            threshold_class.V_MIN = int(file.readline())
            threshold_class.V_MAX = int(file.readline())
        except:
            print("Maze Camera: failed to read in file")

    def __write_values_to_file(self, file, threshold_class):
        file.write("{}\n{}\n{}\n{}\n{}\n{}\n".format(threshold_class.H_MIN, threshold_class.H_MAX, threshold_class.S_MIN,
                                                  threshold_class.S_MAX, threshold_class.V_MIN, threshold_class.V_MAX))

    # Dummy Function used for the createTrackbars function
    def __on_trackbar(self, x):
        pass

    # Creates trackbars used for calibrating filter settings
    def __createTrackbars(self, thresh, winname):
        cv2.namedWindow(winname)

        cv2.createTrackbar("H_MIN", winname, thresh.H_MIN, 255, self.__on_trackbar)
        cv2.createTrackbar("H_MAX", winname, thresh.H_MAX, 255, self.__on_trackbar)
        cv2.createTrackbar("S_MIN", winname, thresh.S_MIN, 255, self.__on_trackbar)
        cv2.createTrackbar("S_MAX", winname, thresh.S_MAX, 255, self.__on_trackbar)
        cv2.createTrackbar("V_MIN", winname, thresh.V_MIN, 255, self.__on_trackbar)
        cv2.createTrackbar("V_MAX", winname, thresh.V_MAX, 255, self.__on_trackbar)

    # Displays a window and trackbars that allows the user to calibrate a filter
    def __getThreshold(self, HSV, thresh, name):
        self.__createTrackbars(thresh, name)  # create trackbar window
        font = cv2.FONT_HERSHEY_SIMPLEX  # Font used for image text

        # Keep displaying image until spacebar is pressed
        while (1):
            # Create trackbars for each threshold value
            thresh.H_MIN = cv2.getTrackbarPos('H_MIN', name)
            thresh.H_MAX = cv2.getTrackbarPos('H_MAX', name)
            thresh.S_MIN = cv2.getTrackbarPos('S_MIN', name)
            thresh.S_MAX = cv2.getTrackbarPos('S_MAX', name)
            thresh.V_MIN = cv2.getTrackbarPos('V_MIN', name)
            thresh.V_MAX = cv2.getTrackbarPos('V_MAX', name)

            # Use threshold values to create a filtered image
            img = cv2.inRange(HSV, np.array([thresh.H_MIN, thresh.S_MIN, thresh.V_MIN]),
                              np.array([thresh.H_MAX, thresh.S_MAX, thresh.V_MAX]))
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

            # Display a fitlered image
            cv2.putText(img, 'Press Spacebar to Continue', (5, 25), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow('image', img)

            k = cv2.waitKey(30)
            if k == 32:
                break

        # Finished, destroy windows
        cv2.destroyWindow('image')
        cv2.destroyWindow(name)

    # This function will go through each filter and allow the user to calibrate the threshold values
    def calibrate_filters(self):
        if self.noCam:
            img = cv2.imread(NOCAM_IMG,cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()
        else:
            print("Maze Camera: Calibrate filters error: camera not open")
            return

        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.__getThreshold(HSV, self.wallsThreshold, "WALLS")
        self.__getThreshold(HSV, self.endPointThreshold, "End Point")
        self.__getThreshold(HSV, self.spheroThreshold, "Sphero (Make Sure it is not glowing)")


# --------------------------------- Camera and Filter Images ----------------------------------------------------------#
    # Returns a raw, unfiltered image. transform is a bool flag, if true it will return a transformed image based on the
    # corners
    def get_image_unfiltered(self, transform=False):
        if self.noCam:
            img = cv2.imread(NOCAM_IMG,cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()
        else:
            return
        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except:
                print("Maze Camera: Get Image Error: failed to get transformed image")
        return img

    # Returns an image filtered with the threshold values for the walls
    # transform is a bool flag, if true it will return a transformed image based on the
    # corners
    def get_image_wall_filtered(self, transform=False):
        if self.noCam:
            img = cv2.imread(NOCAM_IMG,cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()

        else:
            return

        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except:
                print("Maze Camera: Get Image Error: failed to get transformed image")

        # Convert it to HSV and fitler it
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        walls_img = cv2.inRange(HSV, np.array([self.wallsThreshold.H_MIN, self.wallsThreshold.S_MIN, self.wallsThreshold.V_MIN]),
                                np.array([self.wallsThreshold.H_MAX, self.wallsThreshold.S_MAX, self.wallsThreshold.V_MAX]))
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_OPEN, kernel)
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_CLOSE, kernel)

        return walls_img

    # Returns an image filtered with the threshold values for the endpoint
    # transform is a bool flag, if true it will return a transformed image based on the
    # corners
    def get_image_endpoint_filtered(self, transform=False):
        if self.noCam:
            img = cv2.imread(NOCAM_IMG,cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()

        else:
            return

        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except:
                print("Maze Camera: Get Image Error: failed to get transformed image")

        # Convert it to HSV and filter it
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        endPoint_img = cv2.inRange(HSV, np.array([self.endPointThreshold.H_MIN, self.endPointThreshold.S_MIN, self.endPointThreshold.V_MIN]),
                                   np.array([self.endPointThreshold.H_MAX, self.endPointThreshold.S_MAX, self.endPointThreshold.V_MAX]))
        endPoint_img = cv2.morphologyEx(endPoint_img, cv2.MORPH_OPEN, kernel)
        endPoint_img = cv2.morphologyEx(endPoint_img, cv2.MORPH_CLOSE, kernel)

        return endPoint_img

    # Returns an image filtered with the threshold values for the sphero
    # transform is a bool flag, if true it will return a transformed image based on the
    # corners
    def get_image_sphero_filtered(self, transform=False):
        if self.noCam:
            img = cv2.imread(NOCAM_IMG,cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()

        else:
            return

        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except:
                print("Maze Camera: Get Image Error: failed to get transformed image")

        # Convert it to HSV and filter it
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        sphero_img = cv2.inRange(HSV, np.array([self.spheroThreshold.H_MIN, self.spheroThreshold.S_MIN, self.spheroThreshold.V_MIN]),
                                   np.array([self.spheroThreshold.H_MAX, self.spheroThreshold.S_MAX, self.spheroThreshold.V_MAX]))
        sphero_img = cv2.morphologyEx(sphero_img, cv2.MORPH_OPEN, kernel)
        sphero_img = cv2.morphologyEx(sphero_img, cv2.MORPH_CLOSE, kernel)

        return sphero_img

# --------------------------------- Setting and Getting Corners ----------------------------------------------------------#
    # Get corners returns a list of corner coordinates
    def get_corners(self):
        if self.corners_set:
            return self.corners

    # This function will write the current corner values to a text file
    def __save_corners(self):
        if self.corners_set:
            print("Maze Camera: Saving corner values to file")
            with open(CORNERS_FILE, "w") as f:
                json.dump((self.maze_ROI, self.corners), f)



    # This function will read in corner values from a text file
    def __load_corners(self):
        try:
            with open(CORNERS_FILE) as f:
                self.maze_ROI, self.corners = json.load(f)
            print("Maze Camera: Loading previous corner values from file: ", self.corners)
            self.corners_set = True
        except:
            print("Maze Camera: Unable to load corner data from corners.txt. Please set corners before running maze")

    # Set the corners used for the maze
    def set_corners(self):
        # Collect a raw image of the maze
        corners_img = self.get_image_unfiltered()

        self.corners_set = False  # Set to false to lockdown get_corners function
        old_corners = self.corners
        self.corners = []   # Reset corners

        # Put instructions on image before showing it
        cv2.putText(corners_img, 'Click on maze corners; press space to skip', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 255),
                    2)
        cv2.imshow('Add Corners', corners_img)  # Show an image for the user to reference
        cv2.setMouseCallback('Add Corners', self.CallBackFunc)  # Allow user to click on corners
        # Get the 4 corners from user input
        while(len(self.corners) < 4):
            k = cv2.waitKey(10)
            if k == 32: #cancel the function
                self.corners_set = True
                self.corners = old_corners
                cv2.destroyWindow('Add Corners')
                return

        # Store the X and Y pixel boundaries bound by the corners in the maze_ROI dictionary
        self.maze_ROI['row1'] = int(min(y for x, y in self.corners))
        self.maze_ROI['row2'] = int(max(y for x, y in self.corners))
        self.maze_ROI['col1'] = int(min(x for x, y in self.corners))
        self.maze_ROI['col2'] = int(max(x for x, y in self.corners))

        # Store an unaltered copy of corners
        unaltered_corners = []
        for p in self.corners:
            unaltered_corners.append((p[0],p[1]))

        # We will sort the corners to get this order:
        # 1st: Upper left hand corner of image
        # 2nd: Upper right hand corner of image
        # 3rd: Lower left hand corner of image
        # 4th: Lower right hand corner of image
        # Essentially the corners should trace out a "Z" on the image

        # Sort the corners based on ascending column (Y) values
        self.corners = sorted(self.corners, key=lambda k: k[1])

        # Sort the upper corners
        if self.corners[0][0] > self.corners[1][0]:
            temp = self.corners[0]
            self.corners[0] = self.corners[1]
            self.corners[1] = temp

        # Sort the lower corners
        if self.corners[2][0] > self.corners[3][0]:
            temp = self.corners[2]
            self.corners[2] = self.corners[3]
            self.corners[3] = temp

        # Set the minimum X value to zero reference, and the minimum Y value to zero reference
        for i in range(len(self.corners)):
            self.corners[i][0] -= int(min(x for x, y in unaltered_corners))
            self.corners[i][1] -= int(min(y for x, y in unaltered_corners))

        self.corners_set = True
        self.__save_corners()
        cv2.destroyWindow('Add Corners')


    # Used by getCorners to select corners by user mouse clicks
    def CallBackFunc(self, event, x, y, flags, userdata):

        if event == cv2.EVENT_LBUTTONDOWN:
            print ("Touched")
            if len(self.corners) >= 4:
                return
            print("Corner Added", [x,y])
            self.corners.append([x,y])

# ------------------------------------------ Transformations ----------------------------------------------------------#

    def __getTransformation(self):
        if not self.corners_set:
            print("Maze Camera: Transform Error: corners not set")

        else:
            transformed_corners = [[0, 0], [PERSPECTIVE_WIDTH - 0, 0], [0, PERSPECTIVE_HEIGHT - 0],
                                   [PERSPECTIVE_WIDTH - 0, PERSPECTIVE_HEIGHT - 0]]

            transformation = cv2.getPerspectiveTransform(np.array(self.corners, np.float32),
                                                         np.array(transformed_corners, np.float32))
            return transformation

    def __crop_image(self,img):
        if self.corners_set:
            # Crop image to that which is within corners
            img = img[self.maze_ROI['row1']: self.maze_ROI['row2'], self.maze_ROI['col1']: self.maze_ROI['col2']]
            # Transform image
            transformation = self.__getTransformation()
            img = cv2.warpPerspective(img, transformation, (PERSPECTIVE_WIDTH, PERSPECTIVE_HEIGHT))
        else:
            print("Maze Camera: Crop Image Error: corners not set")

        return img

# ------------------------------------------ OS CHECKER ---------------------------------------------------------------#
    # This function will determine the OS running the program.  It assumes that either Windows or Linux is running it.
    # Mac OS is not directly supported, but will be assumed to be Linux.  The result of this test will determine which
    # parameters to use for camera brightness and exposure.
    def __check_os(self):
        if os.name == 'nt':
            print("Windows OS Found")
            self.cam_brightness_init = CAM_INITIAL_BRIGHTNESS_WIN
            self.cam_exposure_init = CAM_INITIAL_EXPOSURE_WIN
            self.cam_brightness_min = CAM_MIN_BRIGHTNESS_WIN
            self.cam_brightness_max = CAM_MAX_BRIGHTNESS_WIN
            self.cam_exposure_min = CAM_MIN_EXPOSURE_WIN
            self.cam_exposure_max = CAM_MAX_EXPOSURE_WIN
            return 'Windows'
        else:
            print("Linux OS Found")
            self.cam_brightness_init = CAM_INITIAL_BRIGHTNESS_LINUX
            self.cam_exposure_init = CAM_INITIAL_EXPOSURE_LINUX
            self.cam_brightness_min = CAM_MIN_BRIGHTNESS_LINUX
            self.cam_brightness_max = CAM_MAX_BRIGHTNESS_LINUX
            self.cam_exposure_min = CAM_MIN_EXPOSURE_LINUX
            self.cam_exposure_max = CAM_MAX_EXPOSURE_LINUX
            return 'Linux'
