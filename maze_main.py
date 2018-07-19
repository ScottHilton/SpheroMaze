#####################################################################
# Brigham Young University
# Sphero Maze Runner
# ECEn Department Demo
# Camera Main Code
#
#####################################################################

# Written in Python 3
# Version 0.0 (Prototype)
# About this version
# July 3, 2018
# 1.  Prototype for the main block of the Sphero Maze Runner
DEBUG_NO_SPHERO = False
DEBUG_NO_CAM = True

if DEBUG_NO_SPHERO:
    print("No Sphero")
    import sphero_driver_dummy as sphero_driver
else:
    import sphero_driver

from camera_main import Maze_Camera
from solver import Maze_Solver
from controller_main import Maze_Controller
from GUI_main import Main_Window, Calibrate_Window, PID_Window
import tkinter as tk
import threading
import cv2
import time
import sys

class Application:
    def __init__(self):
        # Sphero Object
        self.sphero = sphero_driver.Sphero

        # Maze Camera
        self.camera = Maze_Camera(nocam = DEBUG_NO_CAM)

        # Maze Solver
        self.maze_solver = Maze_Solver(self.camera)

        # Controller
        self.controller = Maze_Controller(self.maze_solver)

        # Master frame for GUI's
        self.root = tk.Tk()
        self.GUI = Main_Window(self.root, self)

        # Live feed flag
        self.live_feed = False
        self.destroy_feed_window = False

        # Flags
        self.running = True
        self.sphero_connected = False
        self.sphero_connecting = False
        self.sphero_orienting = False
        self.calibrating_filters = False

    def run(self):
        counter = 0
        while self.running:

            # Update GUI
            try:
                self.root.update_idletasks()
                self.root.update()
                self.GUI.update_status_indicators()
            except:
                print("GUI not found")
                exit()

            # Live feed
            if self.live_feed:
                image = self.camera.get_image_unfiltered()
                cv2.imshow("Live Feed", image)
                cv2.waitKey(10)
            if self.destroy_feed_window:
                cv2.destroyWindow("Live Feed")
                cv2.waitKey(10)
                self.destroy_feed_window = False

            # Thread Status
            counter += 1
            if counter >= 50000:
                #print(threading.enumerate())
                counter = 0

    ### Live Feed ###
    def toggle_live_feed(self):
        self.live_feed = not self.live_feed

        if self.live_feed == False:
            self.destroy_feed_window = True

    ### Sphero Commands ###

    # Connect to a Sphero device.
    def sphero_connect(self, sphero_name):
        # Function to connect to Sphero
        def connect():
            self.sphero_connecting = True  # To prevent multiple instances of trying to connect
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

            # Try twice to connect to a sphero, if this fails return out of function
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

            # Connection was succesful, set up Sphero device
            self.sphero.set_stablization(0, False)  # Lock Gyro for orienting
            self.sphero.set_rgb_led(0, 255, 0, 0, False)  # Sphero Color
            self.sphero.set_back_led(255, False)  # Orienting LED

            time.sleep(5)  # pause so user can orient Sphero as desired (blue LED shows the back of ball)
            self.sphero.set_heading(0, False)  # Set heading
            self.sphero.set_stablization(1, False)  # Unlock gyro
            self.sphero.set_back_led(0, False)  # Turn off orienting LED
            self.sphero.set_rgb_led(0, 255, 0, 0, False)  # Set Sphero Color

            self.sphero_connected = True  # Set flag
            print("Sphero Connected")
            self.GUI.sphero_connection_changed()  # Tell GUI to update
            self.sphero_connecting = False

        # Make a thread to connect to the Sphero
        # Create and start thread for connecting to Sphero
        if not self.sphero_connecting or not self.sphero_connected:
            sphero_thread = threading.Thread(target=connect,name="Sphero_Thread")
            sphero_thread.daemon = True
            sphero_thread.start()

    # Disconnect from a connected Sphero device.  Do not disconnect if orienting
    def sphero_disconnect(self):
        # Disconnect if
        if self.sphero_connected and not self.sphero_orienting:  # Check if there is a Sphero connected
            self.sphero.set_back_led(0, False)  # Turn off blue orienting led
            self.sphero.disconnect()  # Disconnect from Sphero
            self.GUI.sphero_connection_changed()  # Trigger update
            self.sphero_connected = False  # Change flag
            print("Sphero Disconnect")

    # Locks Sphero gyro, lights LED on Sphero for orienting the Sphero's heading
    def sphero_orient(self):
        # Function for the orient thread to run
        def orient():
            if not self.sphero_orienting:  # Make sure it is not already orienting
                self.sphero_orienting = True  # Set flag to lock function
                self.sphero.set_stablization(0, False)  # Lock Sphero gyros
                self.sphero.set_back_led(255, False)  # Light up blue orienting led
                time.sleep(5)  # Wait five seconds
                self.sphero.set_heading(0, False)  # Reset heading to direction of blue led
                self.sphero.set_stablization(1, False)  # Unlock Sphero gyros
                self.sphero.set_back_led(0, False)  # Turn off blue led
                self.sphero_orienting = False  # Set flag to unlock function

        # Try to orient the Sphero if it is connected
        if self.sphero_connected:
            # Create and start orient thread
            orient_thread = threading.Thread(target=orient,name="Orient Sphero")
            orient_thread.daemon
            orient_thread.start()
            print("Sphero Orient")

    ### Calibration Commands ###
    def calibrate_corners(self):
        print("Select Corners")
        self.camera.set_corners()
        pass  # Call camera set corners command

    def calibrate_filters(self):
        print("Calibrate Filters")  # Call camera calibrate command
        self.calibrating_filters = True
        self.camera.calibrate_filters()
        self.camera.save_threshold_values()
        self.calibrating_filters = False
        self.camera.set_corners()

    def calibrate_camera(self):
        print("Calibrate Camera")  # Create Settings GUI for camera exposure and brightness
        self.newWindow_camera_settings = tk.Toplevel(self.root)
        self.settings_window = Calibrate_Window(self.newWindow_camera_settings, self.camera)

    ### Maze Solver Commands ###
    def maze_start(self):
        def start():
            self.controller.control_start()
            self.controller.navigate_maze(self.sphero)

        if not self.sphero_connected:
            print("Connect a Sphero First")
            return
        else:
            controller_thread = threading.Thread(target=start,name="Maze Solver Controller")
            controller_thread.daemon = True
            controller_thread.start()
            print("Start Maze")


    def maze_stop(self):
        self.controller.control_stop()
        print("Stop Maze")

    ### Controller Commands ###
    def controller_set_PID(self):
        print("Set PID") # Create PID GUI
        self.newWindow_pid_settings = tk.Toplevel(self.root)
        self.pid_window = PID_Window(self.newWindow_pid_settings, self.controller)

    ### Exit program  ###
    def quit_program(self):
        # Clean up flags
        self.running = False
        cv2.destroyAllWindows()
        time.sleep(0.2)
        sys.exit()

if __name__ == '__main__':
    main = Application()
    main.run()
