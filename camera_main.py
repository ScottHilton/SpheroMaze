"""
Brigham Young University
Sphero Maze Runner
ECEn Department Demo
Camera Main Code

Written in Python3
August 2019
"""

# Imports
import cv2
import time
import numpy as np
import json
import os
import tkinter as tk
import PIL.Image
import PIL.ImageTk

"""
Save Files:

these macros contain the file names for text files where various information
is saved.
Parameters:  saves threshold values for filters
Corners:  saves coordinates for four corners of maze. 
Camera settings:  saves camera settings (exposure/brightness)
No Camera Img:  Image of a dummy maze used in testing/debugging 
"""
PARAMETERS_FILE = "parameters.txt"
PRESET_FILE = "preset_settings.txt"
CORNERS_FILE = "corners.txt"
CAMERA_SETTINGS_FILE = "camSettings.txt"
NOCAM_IMG = 'testImage.jpg'

"""
Camera Parameters and Settings:

the following macros are used in operating the camera. 

The camera number indicates which camera is being used; default value is 0. If
there are multiple cameras connected to a computer, adjust this integer 
value until you get the correct camera.  

The maximum, minimum, and initial values for brightness and exposure are given.
Be aware that for OpenCV that Linux and Windows use different values. Also, 
these values may change with different camera, so double-check these values for
each camera (along with camera resolution).  

The initial values are used to set up the camera.  
"""
CAMERA_NUMBER = 0
# CAM_MAX_EXPOSURE = 20000
# CAM_MAX_BRIGHTNESS = 100.0
# CAM_INITIAL_BRIGHTNESS = 100
# CAM_INITIAL_EXPOSURE = 10

# Linux Camera Parameters
CAM_MAX_EXPOSURE_LINUX = 200
CAM_MAX_BRIGHTNESS_LINUX = 100.0
CAM_MIN_EXPOSURE_LINUX = 0
CAM_MIN_BRIGHTNESS_LINUX = 0
CAM_INITIAL_BRIGHTNESS_LINUX = 100
CAM_INITIAL_EXPOSURE_LINUX = 50
# Windows Camera Parameters
CAM_MAX_EXPOSURE_WIN = 0
CAM_MAX_BRIGHTNESS_WIN = 50
CAM_MIN_EXPOSURE_WIN = -13
CAM_MIN_BRIGHTNESS_WIN = 0
CAM_INITIAL_BRIGHTNESS_WIN = 25
CAM_INITIAL_EXPOSURE_WIN = -10

# Pixel width and height for cropped and warped maze image
PERSPECTIVE_WIDTH = 560		# Pixel Width
PERSPECTIVE_HEIGHT = 240    # Pixel Height

"""
Filtering parameters

The kernel is used for filtering. The trackbar maximums keep the HSV limited to 
legimate HSV values.
"""
kernel = np.ones((3, 3), np.uint8)  # Kernel for filtering
H_TRACKBAR_MAX = 180
S_TRACKBAR_MAX = 255
V_TRACKBAR_MAX = 255


# This class will handle the camera and the filters
class MazeCamera:
    """Maze Camera: interface for camera

        The purpose of this code is to provide an interface for the maze
        code to interact with the camera. This code will handle the camera
        object, camera settings, and filter settings. This code will also
        handle functionality for corner coordinates.

    """

    def __init__(self, nocam=False):
        """Initialize the camera and color filters.

        First, the nocam flag is checked to determine whether to open the camera
        or load the dummy maze image. Next, the camera parameter variables are
        initialized according the the OS.

        The camera is then opened and the filters initialized.  Lastly, the
        corner coordinates from the previous run are loaded.

        :param nocam: boolean; if true, an image of the maze is loaded in place
                      of live stream (testing/debugging).
        """
        # Flag for no camera debug mode
        self.noCam = nocam

        # Camera Parameter setup
        self.cam_brightness_min = CAM_MIN_BRIGHTNESS_LINUX
        self.cam_brightness_max = CAM_MAX_BRIGHTNESS_LINUX
        self.cam_exposure_min = CAM_MIN_EXPOSURE_LINUX
        self.cam_exposure_max = CAM_MAX_EXPOSURE_LINUX
        self.cam_exposure_value = CAM_INITIAL_EXPOSURE_LINUX
        self.cam_brightness_value = CAM_INITIAL_BRIGHTNESS_LINUX
        self.OS = self.__check_os()

        # Camera Setup
        self.camera_open = False   # True if the camera is open
        self.camera_setup = False  # True if camera settings are configured
        if not self.noCam:
            self.cap = self.__open_camera()
            self.__setup_camera()

        # Filter setup
        # Initialize the thresholds for walls and endPoints
        self.wallsThreshold = self.Threshold("Walls Threshold")
        self.endPointThreshold = self.Threshold("Endpoint Threshold")
        self.__init_thresholds()

        # Load corners
        self.corners_set = False
        self.__load_corners()

    def __del__(self):
        """Class destructor releases camera at close of program; resets flags

        :return: nothing
        """
        try:
            if self.cap:
                self.cap.release()
            print("Maze Camera: Closing Camera")
        except Exception as error:
            print("Maze Camera: Hard Close")
            print(error)

        # Reset flags
        self.camera_open = False
        self.camera_setup = False

# --------------------------  Helper Classes --------------------------------- #

    class Threshold:
        """
        The HSV threshold class used for storing filter thresholds values for
        each filter

        These values will be overwritten with the actual filter values
        """
        def __init__(self, threshold_name):
            self.name = threshold_name  # Identifying name
        # HSV values
        H_MIN = 0
        H_MAX = 180
        S_MIN = 0
        S_MAX = 255
        V_MIN = 0
        V_MAX = 255

    class Sliders:
        def __init__(self, thresh, trackbar_frame):
            """ Creates sliders for adjusting HSV values

            :param thresh: threshold class
            :param trackbar_frame: tk frame for the trackbar
            :return: HSV min/max values for the threshold class
            """
            self.h_min = self.__init_trackbar(trackbar_frame, 'H Min',
                                              H_TRACKBAR_MAX, thresh.H_MIN)
            self.h_max = self.__init_trackbar(trackbar_frame, 'H Max',
                                              H_TRACKBAR_MAX, thresh.H_MAX)
            self.s_min = self.__init_trackbar(trackbar_frame, 'S Min',
                                              S_TRACKBAR_MAX, thresh.S_MIN)
            self.s_max = self.__init_trackbar(trackbar_frame, 'S Max',
                                              S_TRACKBAR_MAX, thresh.S_MAX)
            self.v_min = self.__init_trackbar(trackbar_frame, 'V Min',
                                              V_TRACKBAR_MAX, thresh.V_MIN)
            self.v_max = self.__init_trackbar(trackbar_frame, 'V Max',
                                              V_TRACKBAR_MAX, thresh.V_MAX)

        @staticmethod
        def __init_trackbar(trackbar_frame, label, max_value, init_value):
            """

            :param trackbar_frame: Tkinter trackbar frame
            :param label: Name of trackbar
            :param max_value: Maximum value of slider
            :param init_value: Initial value for slider
            :return: tkinter scale/trackbar/slider object
            """
            trackbar = tk.Scale(trackbar_frame, label=label,
                                from_=0, to=max_value,
                                orient='horizontal')
            trackbar.set(init_value)
            trackbar.pack()
            return trackbar

# ------------------------------------- Camera Setup ------------------------- #

    def __open_camera(self):
        """ Open camera will open up camera object (video capture).

        :return: camera object if successful; None otherwise
        """
        # Open the camera
        try:
            camera = cv2.VideoCapture(CAMERA_NUMBER)
            # Check if the opened
            if camera.isOpened():
                self.camera_open = True
                print("Maze Camera Opened")
                return camera
            else:
                print("Maze Camera: Init Error: Failed to open camera")
                return
        # Sound the alarm if the camera fails to open
        except Exception as error:
            print("Maze Camera: Init Error: Failed to open the camera for the",
                  " following reason:")
            print(error)
            return

    def __setup_camera(self):
        """ Setup camera will initialize the settings for the camera

        The camera brightness and exposure values are set to initial values
        according to the OS

        :return: nothing
        """
        # Initialize camera brightness and exposure
        self.cam_brightness_value = self.cam_brightness_init
        self.cam_exposure_value = self.cam_exposure_init
        # Apply exposure and brightness values to camera
        if self.camera_open:
            try:
                # This is needed in Linux to allow changes in exposure
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                self.__load_cam_settings()
                # Windows
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS,
                                 self.cam_brightness_value)
                    self.cap.set(cv2.CAP_PROP_EXPOSURE,
                                 self.cam_exposure_value)
                # Linux
                else:
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS,
                                 (self.cam_brightness_value /
                                  CAM_MAX_BRIGHTNESS_LINUX))
                    self.cap.set(cv2.CAP_PROP_EXPOSURE,
                                 (self.cam_exposure_value /
                                  CAM_MAX_EXPOSURE_LINUX))
                self.camera_setup = True  # Set flag
            except Exception as error:
                print("Maze Camera Error:  Failed to setup camera: ")
                print(error)

# -------------------- Reset functions for camera and filters ---------------- #
    def reset_camera(self):
        """ The reset camera function will close the camera, and then reopen
         the camera

        The reset process is as following:
        1. Release the camera
        2. Open camera again
        3. Setup camera again
        4. Pause for a moment
        :return: nothing
        """
        try:
            self.cap.release()
            self.__open_camera()
            self.__setup_camera()
            time.sleep(0.1)
            print("Maze Camera Reset")
        except Exception as error:
            print("Maze Camera failed to reset: ")
            print(error)

# ------------------------------ Set Brightness and Exposure ----------------- #
    def set_exposure(self, x):
        """ Set camera exposure

        :param x: integer value to set the exposure to
        :return: nothing
        """
        if self.camera_open and self.camera_setup:
            try:
                self.cam_exposure_value = int(x)
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, int(x))
                else:
                    self.cap.set(cv2.CAP_PROP_EXPOSURE,
                                 int(x) / CAM_MAX_EXPOSURE_LINUX)
            except Exception as error:
                print("Maze Camera: set_exposure error")
                print(error)
        else:
            print("Maze Camera: set_exposure error: camera not ready")

    def set_brightness(self, x):
        """ Set camera brightness

        :param x: integer value to set the brightness to
        :return: nothing
        """
        if self.camera_open and self.camera_setup:
            try:
                self.cam_brightness_value = int(x)
                if self.OS == 'Windows':
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS, int(x))
                else:
                    self.cap.set(cv2.CAP_PROP_BRIGHTNESS,
                                 int(x) / CAM_MAX_BRIGHTNESS_LINUX)
            except Exception as error:
                print("Maze Camera: set_brightness error")
                print(error)
        else:
            print("Maze Camera: set_brightness error: camera not ready")

# ------------------------------ Save Brightness and Exposure ---------------- #
    def save_cam_settings(self):
        """ This function will write the current camera settings to a text file

        :return: nothing
        """
        print("Maze Camera: Saving camera settings to file")
        with open(CAMERA_SETTINGS_FILE, "w") as f:
            json.dump((self.cam_brightness_value, self.cam_exposure_value), f)

    def __load_cam_settings(self):
        """ This function will read in camera settings from a text file

        :return: nothing
        """
        try:
            with open(CAMERA_SETTINGS_FILE) as f:
                self.cam_brightness_value, self.cam_exposure_value =\
                    json.load(f)
            print("Maze Camera: Loading previous brightness and exposure",
                  " settings from file: ",
                  self.cam_brightness_value,
                  self.cam_exposure_value)
        except Exception as error:
            print("Maze Camera: Unable to load brightness and exposure settings"
                  " from camSettings.txt.")
            print(error)

# -------------------- Filters and Stuff ------------------------------------- #
    def __init_thresholds(self):
        """ This function will read in threshold values from a text file

        :return:
        """
        print("Maze Camera: Loading filter values from param file")
        f = open(PARAMETERS_FILE, 'r')
        self.__read_values_from_file(f, self.wallsThreshold)
        self.__read_values_from_file(f, self.endPointThreshold)

        f.close()

    def save_threshold_values(self):
        """ This function will write the current threshold values to a text file

        :return:
        """
        print("Maze Camera: Saving filter values to param file")
        f = open(PARAMETERS_FILE, 'w')
        self.__write_values_to_file(f, self.wallsThreshold)
        self.__write_values_to_file(f, self.endPointThreshold)

        f.close()

    @staticmethod
    def __read_values_from_file(file, threshold_class):
        """ Supporting function that reads values into a threshold class.

        :param file: file object to be read
        :param threshold_class: the threshold class used (e.g. walls)
        :return: nothing
        """
        try:
            threshold_class.H_MIN = int(file.readline())
            threshold_class.H_MAX = int(file.readline())
            threshold_class.S_MIN = int(file.readline())
            threshold_class.S_MAX = int(file.readline())
            threshold_class.V_MIN = int(file.readline())
            threshold_class.V_MAX = int(file.readline())

        except Exception as error:
            print("Maze Camera: failed to read in file: ")
            print(error)

    @staticmethod
    def __write_values_to_file(file, threshold_class):
        """ Writes values to a text file

        :param file: file object to write to
        :param threshold_class: the threshold class used (e.g. walls)
        :return:
        """
        file.write("{}\n{}\n{}\n{}\n{}\n{}\n".format(threshold_class.H_MIN,
                                                     threshold_class.H_MAX,
                                                     threshold_class.S_MIN,
                                                     threshold_class.S_MAX,
                                                     threshold_class.V_MIN,
                                                     threshold_class.V_MAX))

    @staticmethod
    def __on_trackbar(x):
        """ Dummy Function used for the createTrackbars function

        OBSOLETE: replaced by tkinter interface

        :param x: trackbar value
        :return: nothing
        """
        pass

    def __create_trackbars(self, thresh, winname):
        """ Creates trackbars used for calibrating filter settings

        OBSOLETE: replaced by tkinter interface

        :param thresh: Threshold class (e.g. walls)
        :param winname: String for window name
        :return: nothing
        """
        cv2.namedWindow(winname)

        cv2.createTrackbar("H_MIN", winname, thresh.H_MIN,
                           H_TRACKBAR_MAX, self.__on_trackbar)
        cv2.createTrackbar("H_MAX", winname, thresh.H_MAX,
                           H_TRACKBAR_MAX, self.__on_trackbar)
        cv2.createTrackbar("S_MIN", winname, thresh.S_MIN,
                           S_TRACKBAR_MAX, self.__on_trackbar)
        cv2.createTrackbar("S_MAX", winname, thresh.S_MAX,
                           S_TRACKBAR_MAX, self.__on_trackbar)
        cv2.createTrackbar("V_MIN", winname, thresh.V_MIN,
                           V_TRACKBAR_MAX, self.__on_trackbar)
        cv2.createTrackbar("V_MAX", winname, thresh.V_MAX,
                           V_TRACKBAR_MAX, self.__on_trackbar)

    @staticmethod
    def close_window(tk_window):
        """ Closes the tkinter window; used for button command

        :param tk_window: tkinter window to be closed
        :return: sadness, despair, gloom
        """
        tk_window.eval('::ttk::CancelRepeat')
        tk_window.quit()
        tk_window.destroy()

    def __pack_buttons(self, buttons_frame, filter_window, threshold, sliders):
        """ Packs buttons for filter window.

        Packs the continue and load presets buttons

        :param buttons_frame: tkinter frame for buttons
        :param filter_window: tk root window
        :return: nothing
        """
        exit_button = tk.Button(buttons_frame, text='Continue',
                                font=('Arial', 12, 'bold'), fg='blue',
                                command=lambda:
                                self.close_window(filter_window))
        exit_button.pack(side='left', padx=5, pady=5)
        load_button = tk.Button(buttons_frame, text='Load Default',
                                font=('Arial', 12), fg='red',
                                command=lambda:
                                self.__load_preset_values(threshold, sliders))
        load_button.pack(side='left', padx=5, pady=5)

    @staticmethod
    def __load_preset_values(threshold, sliders):
        """ Loads in the preset values into the Threshold class and updates the
        sliders with the new values.

        :param threshold: Threshold class
        :param sliders: Sliders class
        :return: nothing
        """
        try:
            file = open(PRESET_FILE, 'r')

            # Load relevant values
            for line in file:
                if threshold.name in line:
                    hsv_info = line.split(':')[1]
                    hsv_info = hsv_info.split(',')
                    threshold.H_MIN = hsv_info[0]
                    threshold.H_MAX = hsv_info[1]
                    threshold.S_MIN = hsv_info[2]
                    threshold.S_MAX = hsv_info[3]
                    threshold.V_MIN = hsv_info[4]
                    threshold.V_MAX = hsv_info[5]

            # Update trackbars
            sliders.h_min.set(threshold.H_MIN)
            sliders.h_max.set(threshold.H_MAX)
            sliders.s_min.set(threshold.S_MIN)
            sliders.s_max.set(threshold.S_MAX)
            sliders.v_min.set(threshold.V_MIN)
            sliders.v_max.set(threshold.V_MAX)
        except Exception as error:
            print('Failed to load preset values: ', error)

    @staticmethod
    def __pack_image_canvas(hsv_img, image_frame, filter_window):
        """ Create canvas for sample image.

        This tkinter canvas will diplay an image of the maze with the threshold
        values applied. This code initializes the display picture with the raw
        hsv image.

        Note: OpenCV images must be converted to a Tkinter compatible image
         before being displayed. This code does just that.

        :param hsv_img: HSV image from OpenCV
        :param image_frame: Tkinter frame for the image
        :param filter_window: Tkinter main window
        :return: Tkinter canvas for the image
        """
        photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(hsv_img),
                                       master=filter_window)
        img_canvas = tk.Canvas(image_frame,
                               width=PERSPECTIVE_WIDTH,
                               height=PERSPECTIVE_HEIGHT)
        img_canvas.pack(expand=True, fill='both', padx=10, pady=10)
        img_canvas.create_image(0, 0, image=photo, anchor='nw')
        return img_canvas

    def __get_threshold(self, hsv_img, thresh, name):
        """ Displays a window and trackbars that allows the user to calibrate
         a filter

        :param hsv_img: Image in HSV format
        :param thresh: threshold class
        :param name: string for name
        :return: nothing
        """
        print("Threshold Name:", thresh.name)
        # Set up main window
        filter_window = tk.Tk()
        # filter_window.protocol("WM_DELETE_WINDOW", self.close_window)
        filter_window.title(name)

        # Frames
        title_frame = tk.Frame(filter_window)
        top_frame = tk.Frame(filter_window)
        bottom_frame = tk.Frame(filter_window)
        title_frame.pack(side='top')
        top_frame.pack(side='top')
        bottom_frame.pack(side='top')

        image_frame = tk.Frame(top_frame, relief='sunken', borderwidth=5)
        trackbar_frame = tk.Frame(top_frame, relief='sunken', borderwidth=5)
        buttons_frame = tk.Frame(bottom_frame)
        trackbar_frame.pack(side='left', padx=5, pady=5, expand=True, fill='y')
        image_frame.pack(side='left', padx=5, pady=5, expand=True, fill='y')
        buttons_frame.pack(side='bottom')

        # Create Title
        title = tk.Label(title_frame,
                         text=('Adjust ' + name + ' HSV Thresholds'),
                         font=('Arial', 18, 'bold'), fg='blue')
        title.pack()

        # Create sliders
        sliders = self.Sliders(thresh, trackbar_frame)

        # Create Image Canvas
        img_canvas = self.__pack_image_canvas(hsv_img, image_frame,
                                              filter_window)

        # Pack buttoms
        self.__pack_buttons(buttons_frame, filter_window, thresh, sliders)

        try:
            while filter_window.winfo_exists():
                filter_window.update()
                thresh.H_MIN = sliders.h_min.get()
                thresh.H_MAX = sliders.h_max.get()
                thresh.S_MIN = sliders.s_min.get()
                thresh.S_MAX = sliders.s_max.get()
                thresh.V_MIN = sliders.v_min.get()
                thresh.V_MAX = sliders.v_max.get()

                img = cv2.inRange(hsv_img,
                                  np.array([thresh.H_MIN, thresh.S_MIN,
                                            thresh.V_MIN]),
                                  np.array([thresh.H_MAX, thresh.S_MAX,
                                            thresh.V_MAX]))
                img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
                img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
                photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(img),
                                               master=filter_window)
                img_canvas.create_image(0, 0, image=photo, anchor='nw')
        except Exception as error:
            # This avoids a weird tkinter error
            print('Ignoring Adjust Threshold Error:')

    def calibrate_filters(self):
        """ This function will go through each filter and allow the user to
         calibrate the threshold values

        :return: nothing
        """
        img = self.get_image_unfiltered(True)

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.__get_threshold(hsv_img, self.wallsThreshold, "Walls")
        self.__get_threshold(hsv_img, self.endPointThreshold, "End Point")


# --------------------------------- Camera and Filter Images ----------------- #
    #
    def get_image_unfiltered(self, transform=False):
        """ Returns a raw, unfiltered image.

            If transform is true, it will return a transformed image that crops
            out everything outside the region between the corners and forces
            that region into a rectangular shape

        :param transform: Bool; if true, corners are used ot warp image to
        :return: raw image
        """
        if self.noCam:
            img = cv2.imread(NOCAM_IMG, cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()
        else:
            return
        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except Exception as error:
                print("Maze Camera: Get Image Error: failed to get transformed",
                      " image")
                print(error)

        return img

    def get_image_wall_filtered(self, transform=False):
        """ Returns an image filtered with the threshold values for the walls

            If transform is true, it will return a transformed image that crops
            out everything outside the region between the corners and forces
            that region into a rectangular shape

            An image is read from the camera. This image is then passed through
            a threshold filter based on the HSV wall values.

        :param transform: Bool; if true, corners are used ot warp image to
        :return: filtered image of walls
        """
        if self.noCam:
            img = cv2.imread(NOCAM_IMG, cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()

        else:
            return

        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except Exception as error:
                print("Maze Camera: Get Image Error: failed to get transformed",
                      " image")
                print(error)

        # Convert it to HSV and filter it
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        walls_img = cv2.inRange(hsv_img, np.array([self.wallsThreshold.H_MIN,
                                                   self.wallsThreshold.S_MIN,
                                                   self.wallsThreshold.V_MIN]),
                                np.array([self.wallsThreshold.H_MAX,
                                          self.wallsThreshold.S_MAX,
                                          self.wallsThreshold.V_MAX]))
        # walls_img = cv2.erode(walls_img, kernel, iterations=1)
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_OPEN, kernel)
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_OPEN, kernel)
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_CLOSE, kernel)
        # walls_img =cv2.dilate(walls_img, kernel, iterations=2)

        return walls_img

    def get_image_endpoint_filtered(self, transform=False):
        """ Returns an image filtered with the threshold values for the endpoint

        If transform is true, it will return a transformed image that crops
        out everything outside the region between the corners and forces
        that region into a rectangular shape

        An image is read from the camera. This image is then passed through
        a threshold filter based on the HSV endpoint values.

        :param transform: Bool; if true, corners are used ot warp image to
        :return: filtered image of the endpoint
        """
        if self.noCam:
            img = cv2.imread(NOCAM_IMG, cv2.IMREAD_COLOR)

        elif self.camera_open:
            # Read and return image from camera
            ret, img = self.cap.read()

        else:
            return

        # Apply transformation to image if needed
        if transform:
            try:
                img = self.__crop_image(img)
            except Exception as error:
                print("Maze Camera: Get Image Error: failed to get transformed",
                      " image")
                print(error)

        # Convert it to HSV and filter it
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        endpoint_img = cv2.inRange(hsv_img,
                                   np.array([self.endPointThreshold.H_MIN,
                                             self.endPointThreshold.S_MIN,
                                             self.endPointThreshold.V_MIN]),
                                   np.array([self.endPointThreshold.H_MAX,
                                             self.endPointThreshold.S_MAX,
                                             self.endPointThreshold.V_MAX]))
        endpoint_img = cv2.dilate(endpoint_img, kernel, iterations=1)
        endpoint_img = cv2.morphologyEx(endpoint_img, cv2.MORPH_OPEN, kernel)
        endpoint_img = cv2.morphologyEx(endpoint_img, cv2.MORPH_CLOSE, kernel)

        return endpoint_img

# --------------------------------- Setting and Getting Corners -------------- #
    def get_corners(self):
        """ Get corners returns a list of corner coordinates

        :return: list of corners coordinates
        """
        if self.corners_set:
            return self.corners

    def __save_corners(self):
        """ This function will write the current corner values to a text file

        :return: nothing
        """
        if self.corners_set:
            print("Maze Camera: Saving corner values to file")
            with open(CORNERS_FILE, "w") as f:
                json.dump((self.maze_ROI, self.corners), f)

    def __load_corners(self):
        """ This function will read in corner values from a text file

        :return: nothing
        """
        try:
            with open(CORNERS_FILE) as f:
                self.maze_ROI, self.corners = json.load(f)
            print("Maze Camera: Loading previous corner values from file: ",
                  self.corners)
            self.corners_set = True
        except Exception as error:
            print("Maze Camera: Unable to load corner data from corners.txt.",
                  " Please set corners before running maze")
            print(error)

    def set_corners(self):
        """ Set the corners used for the maze

        An image of the maze is collected and displayed to the user. The user
        will click on the four maze corners (physical corners as they appear
        on the image). The coordinates for these four corners is saved.

        The user may cancel this function at anytime by pressing the space bar.

        After selection, the corner coordinates are then used to redefine the
        pixel boundaries of the maze as seen from the current camera position.

        The maze corners are also sorted.
        :return: nothing
        """
        # Collect a raw image of the maze
        corners_img = self.get_image_unfiltered()

        self.corners_set = False  # Set to false to lock get_corners function
        old_corners = self.corners
        self.corners = []   # Reset corners

        # Put instructions on image before showing it
        cv2.putText(corners_img, 'Click on maze corners; press space to skip',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 255), 2)
        if self.noCam:
            cv2.namedWindow('Add Corners', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Add Corners', 403, 302)

        # Show image for the user to reference and allow user to click corners
        cv2.imshow('Add Corners', corners_img)
        cv2.setMouseCallback('Add Corners', self.CallBackFunc)

        # Get the 4 corners from user input
        while len(self.corners) < 4:
            k = cv2.waitKey(10)
            if k == 32:  # cancel the function
                self.corners_set = True
                self.corners = old_corners
                cv2.destroyWindow('Add Corners')
                return

        # Store the X and Y pixel boundaries bound by the corners in the
        # maze_ROI dictionary
        self.maze_ROI['row1'] = int(min(y for x, y in self.corners))
        self.maze_ROI['row2'] = int(max(y for x, y in self.corners))
        self.maze_ROI['col1'] = int(min(x for x, y in self.corners))
        self.maze_ROI['col2'] = int(max(x for x, y in self.corners))

        # Store an unaltered copy of corners
        unaltered_corners = []
        for p in self.corners:
            unaltered_corners.append((p[0], p[1]))

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

        # Set the minimum X value to zero reference, and the minimum Y value
        # to zero reference
        for i in range(len(self.corners)):
            self.corners[i][0] -= int(min(x for x, y in unaltered_corners))
            self.corners[i][1] -= int(min(y for x, y in unaltered_corners))

        self.corners_set = True
        self.__save_corners()
        cv2.destroyWindow('Add Corners')

    def CallBackFunc(self, event, x, y, flags, userdata):
        """ Used by getCorners to select corners by user mouse clicks

        :param event: mouse click event
        :param x: x coordinate of click
        :param y: y coordinate of click
        :param flags: unused, but needed for opencv
        :param userdata: unused, but needed for opencv
        :return: nothing
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Touched")
            if len(self.corners) >= 4:
                return
            print("Corner Added", [x, y])
            self.corners.append([x, y])

# ------------------------------------------ Transformations ----------------- #
    def __getTransformation(self):
        """ Uses corners to transform image perspective

        :return: transformed image
        """
        if not self.corners_set:
            print("Maze Camera: Transform Error: corners not set")

        else:
            transformed_corners = [[0, 0],
                                   [PERSPECTIVE_WIDTH - 0, 0],
                                   [0, PERSPECTIVE_HEIGHT - 0],
                                   [PERSPECTIVE_WIDTH - 0,
                                    PERSPECTIVE_HEIGHT - 0]]

            transformation = \
                cv2.getPerspectiveTransform(np.array(self.corners,
                                                     np.float32),
                                            np.array(transformed_corners,
                                                     np.float32))
            return transformation

    def __crop_image(self, img):
        """ Crops an image according to the corners.

        Uses a warp perspective to return image to rectangular shape

        :param img: Image to be cropped
        :return: cropped image
        """
        if self.corners_set:
            # Crop image to that which is within corners
            img = img[self.maze_ROI['row1']: self.maze_ROI['row2'],
                      self.maze_ROI['col1']: self.maze_ROI['col2']]
            # Transform image
            transformation = self.__getTransformation()
            img = cv2.warpPerspective(img, transformation,
                                      (PERSPECTIVE_WIDTH, PERSPECTIVE_HEIGHT))
        else:
            print("Maze Camera: Crop Image Error: corners not set")

        return img

# ------------------------------------------ OS CHECKER ---------------------- #
    def __check_os(self):
        """ Checks OS

        This function will determine the OS running the program.  It assumes
        that either Windows or Linux is running it.

        Mac OS is not directly supported, but will be assumed to be Linux.

        The result of this test will determine which parameters to use for
         camera brightness and exposure.

        :return: String of OS name
        """
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
