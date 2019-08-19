"""
Brigham Young University
Sphero Maze Runner
ECEn Department Demo
Camera Main Code

Written in Python 3
August 2019
"""

from camera_main import MazeCamera
from solver import MazeSolver
from controller_main import MazeController
from GUI_main import MainWindow, CalibrateWindow, PIDWindow
import tkinter as tk
import threading
import cv2
import time
import sys

DEBUG_NO_SPHERO = False
DEBUG_NO_CAM = False

if DEBUG_NO_SPHERO:
    print("No Sphero Mode")
    import sphero_driver_dummy as sphero_driver
else:
    import sphero_driver


class Application:
    """ This is the main program that runs the Sphero Maze Runner program.

    The main program links together the various parts of the maze (e.g. camera,
    solver, controller, etc) and coordinates their functions.  The program is
    access and controlled by a GUI interface.

    """
    def __init__(self):
        """ Initials each part of the maze, the Sphero, the GUI, and flags.

        Parts of program:
        -Sphero: the Sphero interface
        -Camera: the camera interface; includes filtering functionality
        -Maze Solver: algorithm package for solving maze based in camera inputs
        -Controller: controls the Sphero based on solver and camera inputs
        -GUI: user interface for operating program

        There are three kinds of live feeds:
        1. Live feed: unaltered, unfiltered camera feed
        2. Maze Walls: cropped to corners, filters to maze walls feed; shows
                        possible Sphero paths
        3. Sphero location: cropped to corners, draws a circle around the
                            current location of Sphero in maze.

        Flags used:
        running: True if program is running; False value will terminate program
        sphero_connected: True if a Sphero is connected
        sphero_connecting: True if program is trying to connect to a Sphero;
                            prevents multiple threads from trying to connect.
        sphero_orienting: True if Sphero is being oriented. When orienting the
                          Sphero gyros are locked. This locks out other
                          operations to the Sphero.
        calibrating_filters: True if the filters are being calibrated.
        """
        # Sphero Object
        self.sphero = sphero_driver.Sphero

        # Maze Camera
        self.camera = MazeCamera(nocam=DEBUG_NO_CAM)

        # Maze Solver
        self.maze_solver = MazeSolver(self.camera)

        # Controller
        self.controller = MazeController(self.maze_solver)

        # Master frame for GUI's
        self.root = tk.Tk()
        self.GUI = MainWindow(self.root, self)

        # Live feed flag
        self.live_feed = False
        self.destroy_feed_window = False

        # Maze Debug Flag
        self.maze_feed = False
        self.destroy_maze_window = False

        # Sphero Locator Debug Flag
        self.sphero_feed = False
        self.destroy_sphero_window = False

        # Flags
        self.running = True
        self.sphero_connected = False
        self.sphero_connecting = False
        self.sphero_orienting = False
        self.calibrating_filters = False

    def run(self):
        """ Main driver for the program.

        This driver performs three main tasks:
        1. Update and refresh GUI
        2. Check if there are live feeds active. If so, update them
        3. Check if controller is on (maze is being solved). If not, update
           maze status.
        This driver is run from the main thread. User input from the GUI will
        perform specific, one-time use functions. Often these functions are run
        in separate threads.

        :return: nothing
        """
        while self.running:

            # Update GUI
            try:
                self.root.update_idletasks()
                self.root.update()
                self.GUI.update_status_indicators()
            except Exception as error:
                print("GUI not found:", error)
                exit()

            # Live feed
            if self.live_feed:
                image = self.camera.get_image_unfiltered()
                cv2.imshow("Live Feed", image)
                cv2.waitKey(5)
            if self.destroy_feed_window:
                cv2.destroyWindow("Live Feed")
                cv2.waitKey(5)
                self.destroy_feed_window = False

            # Maze feed
            if self.maze_feed:
                self.maze_solver.findMazeMatrix()
                cv2.imshow("Maze Walls", self.maze_solver.wall_img_debug)
                cv2.waitKey(5)
            if self.destroy_maze_window:
                cv2.destroyWindow("Maze Walls")
                cv2.waitKey(5)
                self.destroy_maze_window = False

            # Sphero location feed
            if self.sphero_feed:
                coordinates = self.maze_solver.getSpheroCorodinates()
                image = self.camera.get_image_unfiltered(True)
                cv2.circle(image, (int(coordinates[0]), int(coordinates[1])),
                           35, (255, 255, 255), 3)
                cv2.imshow("Sphero Position", image)
                cv2.waitKey(5)
            if self.destroy_sphero_window:
                cv2.destroyWindow("Sphero Position")
                cv2.waitKey(5)
                self.destroy_sphero_window = False

            # Maze Status
            if not self.controller.control_status():
                self.update_maze_status(solving=False)

        # Pause a moment to be sure everything is finished.
        time.sleep(0.1)


# ---------------------- Live Feed ------------------------------------------- #
    def toggle_live_feed(self):
        """ Flag is set to toggle live feed or turn it off.  If set to turn off,
        a flag is set to indicate the live feed window should be closed.

        :return: nothing
        """
        self.live_feed = not self.live_feed

        if not self.live_feed:
            self.destroy_feed_window = True

    def toggle_maze_feed(self):
        """ Flag is set to toggle maze wall feed or turn it off.  If set to turn
         off, a flag is set to indicate the maze feed window should be closed.

        :return: nothing
        """
        self.maze_feed = not self.maze_feed

        if not self.maze_feed:
            self.destroy_maze_window = True

    def toggle_sphero_feed(self):
        """ Flag is set to toggle Sphero location feed or turn it off.  If set
        to turn off, a flag is set to indicate the Sphero feed window should be
        closed.

        :return: nothing
        """
        self.sphero_feed = not self.sphero_feed

        if not self.sphero_feed:
            self.destroy_sphero_window = True

# ------------------------ Sphero Commands ----------------------------------- #
    def sphero_connect(self, sphero_name):
        """ Connect to a Sphero device.

        When called, this check if a Sphero is not already connected and if the
        program is currently attempting to connect to a Sphero. If there is no
        Sphero connection and if there no attempt at a connection, then start a
        thread to connect to the Sphero.

        :param sphero_name: String identifier for the target Sphero (e.g. 'BPW')
        :return: nothing
        """
        # Function to connect to Sphero
        def connect():
            """ Run by independent thread: connects to Sphero

            If there is no Sphero already connected and if the selected Sphero
            is valid (chosen in drop down menu), then two attempts will be made
            to connect to the Sphero.

            Once connected, the gyro is locked for a few seconds to give the
            user a moment to orient the Sphero.
            :return: nothing
            """

            self.sphero_connecting = True
            # Check if there is already a Sphero Connected
            if self.sphero_connected:
                print("Sphero Already Connected")
                self.sphero_connecting = False
                return
            # Check if an invalid name for Sphero was given
            elif sphero_name == "":
                print("Please Select a Sphero")
                self.sphero_connecting = False
                return

            # Indicate attempt to connect to Sphero
            print("Connecting to:", sphero_name)
            self.sphero = sphero_driver.Sphero(sphero_name)

            # Try twice to connect to a sphero
            try:
                self.sphero.connect()  # Attempt to connect
            except IOError:
                print("Trying Again")
                try:
                    self.sphero.connect()  # Attempt to connect again
                except IOError:
                    print("Failed to connect to sphero")
                    self.sphero_connecting = False
                    return  # Failed to connect, return out of function

            # Connection was successful, set up Sphero device
            self.sphero.set_stablization(0, False)  # Lock Gyro for orienting
            self.sphero.set_rgb_led(0, 0, 0, 0, False)  # Sphero Color
            self.sphero.set_back_led(255, False)  # Orienting LED

            time.sleep(5)  # pause so user can orient Sphero
            self.sphero.set_heading(0, False)  # Set heading
            self.sphero.set_stablization(1, False)  # Unlock gyro
            self.sphero.set_back_led(0, False)  # Turn off orienting LED
            self.sphero.set_rgb_led(0, 0, 0, 0, False)  # Set Sphero Color

            self.sphero_connected = True  # Set flag
            print("Sphero Connected")
            self.GUI.sphero_connection_changed()  # Tell GUI to update
            self.sphero_connecting = False

        # Make a thread to connect to the Sphero
        # Create and start thread for connecting to Sphero
        if not self.sphero_connecting or not self.sphero_connected:
            sphero_thread = threading.Thread(target=connect,
                                             name="Sphero_Thread")
            sphero_thread.daemon = True
            sphero_thread.start()

    def sphero_disconnect(self):
        """ Disconnect from a connected Sphero device.

        Do not disconnect if orienting.  Turn off LED's and update flags

        :return: nothing
        """
        # Disconnect if
        if self.sphero_connected and not self.sphero_orienting:
            self.sphero.set_back_led(0, False)    # Turn off blue orienting led
            self.sphero.disconnect()
            self.GUI.sphero_connection_changed()  # Trigger update
            self.sphero_connected = False
            print("Sphero Disconnect")

    def sphero_orient(self):
        """ Locks Sphero gyro, lights LED on Sphero for orienting the Sphero's
         heading.

        When called, if there is a Sphero connected, a thread is started which
        runs the orientation procedure:
        1. Locks the gyro
        2. Light up the back LED indicating the Sphero's heading
        3. Wait 5 seconds
        4. Reset heading
        5. Unlock gyro
        6. Turn off back LED
        7. Reset flags
        :return:
        """
        # Function for the orient thread to run
        def orient():
            if not self.sphero_orienting:
                self.sphero_orienting = True  # Set flag to lock function
                self.sphero.set_stablization(0, False)
                self.sphero.set_back_led(255, False)
                time.sleep(5)
                self.sphero.set_heading(0, False)
                self.sphero.set_stablization(1, False)
                self.sphero.set_back_led(0, False)
                self.sphero_orienting = False  # Set flag to unlock function

        # Try to orient the Sphero if it is connected
        if self.sphero_connected:
            # Create and start orient thread
            orient_thread = threading.Thread(target=orient,
                                             name="Orient Sphero")
            orient_thread.daemon = True
            orient_thread.start()
            print("Sphero Orient")

# ----------------------- Calibration Commands ------------------------------- #
    def calibrate_corners(self):
        """ Allows user to set the corners.

        :return: nothing
        """
        print("Select Corners")
        self.camera.set_corners()

    def calibrate_filters(self):
        """ Creates GUI for calibrating the filters.

        Filters are set for walls and endpoint. These values are then saved.

        :return: nothing
        """
        if not self.calibrating_filters:
            print("Calibrate Filters")
            self.calibrating_filters = True
            self.camera.calibrate_filters()
            self.camera.save_threshold_values()
            self.calibrating_filters = False
        else:
            print("Already Calibrating Filters")

    def calibrate_camera(self):
        """ Create Settings GUI for camera exposure and brightness

        :return: nothing
        """
        print("Calibrate Camera")  #
        window_camera_settings = tk.Toplevel(self.root)
        CalibrateWindow(window_camera_settings, self.camera)

# ----------------------- Maze Solver Commands ------------------------------- #
    def maze_start(self):
        """ Begins solving the maze.

        A thread is started specifically to solve the maze.

        :return: nothing
        """
        def start():
            self.controller.control_start()
            self.controller.navigate_maze(self.sphero)

        if not self.sphero_connected:
            print("Connect a Sphero First")
            return
        else:
            self.update_maze_status(solving=True)
            controller_thread = threading.Thread(target=start,
                                                 name="Maze Solver Controller")
            controller_thread.daemon = True
            controller_thread.start()
            print("Start Maze")

    def maze_stop(self):
        """ Stop the maze solving.

        :return: nothing
        """
        self.update_maze_status(solving=False)
        self.controller.control_stop()
        print("Stop Maze")

    def update_maze_status(self, solving=False):
        """ Updates the GUI's maze status label

        :param solving: Flags indicating maze solving status
        :return:
        """
        if solving:
            self.GUI.maze_settings_display.\
                configure(fg="green", text="Maze Status: SOLVING MAZE")
        else:
            try:
                self.GUI.maze_settings_display.\
                    configure(fg="red", text="Maze Status: STOPPED")
            except Exception as error:
                print("Update Maze Display error:", error)

# ------------------------- Controller Commands ------------------------------ #
    def controller_set_pid(self):
        """ Creates GUI to adjust PID gain values

        :return: nothing
        """
        print("Set PID")
        window_pid_settings = tk.Toplevel(self.root)
        PIDWindow(window_pid_settings, self.controller)

# ---------------------------- Exit program  --------------------------------- #
    def quit_program(self):
        """ Exits program

        Sets the running flag to false to stop main thread. All cv2 windows are
        closed.  System is exited.
        :return: nothing
        """
        self.running = False
        cv2.destroyAllWindows()
        time.sleep(0.2)
        sys.exit()


# --------------------------- Main Program ----------------------------------- #
if __name__ == '__main__':
    main = Application()
    main.run()
