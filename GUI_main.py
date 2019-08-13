"""
Brigham Young University
Sphero Maze Runner
ECEn Department Demo
GUI Main Code

Written in Python 3
August 2019
"""

import time
import tkinter as tk
from tkinter import OptionMenu


class MainWindow:
    """
    This is the main user interface menu. It creates interface that calls
     functions from maze_main.py.  It consists of four main sections:
    1. Sphero Connection
        Connect, disconnect, and connection status. Also option for orienting
        the Sphero.
    2. Settings
        Adjust settings for filters, corners, camera, and PID controller
    3. Tests
        Various tests: check filter performance for the maze walls,
         view live feed, or see current Sphero location
    4. Maze Status
        Shows maze status; allows user to start or stop solving maze.

    """
    def __init__(self, master, application):
        """ Initializes the main GUI.

        Flags are created and initialized, frames are created and packed,
        widgets are created and packed.

        Flags used:
        -sphero_device_connected: True if a sphero device connected
        -sphero_status: This will hold the sphero status
        -sphero_connecting: Used to prevent two threads from trying to connect
                            to sphero
        -widgets_setup: True if the widgets have been setup
        -sphero_device: String variable for which sphero is connected (eg WPW)

        -update_sphero_connection: True if sphero is connected
        -update_maze_solving: True if the maze if being solved
        :param master: tkinter root
        :param application:  Main application class
        """
        self.app = application
        self.master = master
        # Variables and flags
        self.sphero_device_connected = False
        self.sphero_status = tk.StringVar()
        self.sphero_status_display = tk.Label()  # Label for status
        self.sphero_connecting = False
        self.widgets_setup = False
        self.sphero_device = tk.StringVar()
        self.maze_settings_display = tk.Label()

        # Initialize window, frames, pack main frames
        self.create_main_window()
        self.gui_frames = self.Frames(self.master)

        # Create the widgets
        self.create_widgets()

        # Status Flags - Used for updating status indicators on GUI
        self.update_sphero_connection = False
        self.update_maze_solving = False

    class Frames:
        def __init__(self, master):
            """
            Create frames and pack them up
            """
            self.frame_inner_title = tk.Frame(master)
            self.frame_sphero = tk.Frame(master, relief="sunken",
                                         borderwidth=5)
            self.frame_settings = tk.Frame(master, relief="sunken",
                                           borderwidth=5)
            self.frame_tests = tk.Frame(master, relief="sunken",
                                        borderwidth=5)
            self.frame_maze_settings = tk.Frame(master, relief="sunken",
                                                borderwidth=5)
            self.frame_quit = tk.Frame(master, relief="sunken",
                                       borderwidth=5)

            self.frame_inner_title.pack(fill="x", expand=True)
            self.frame_sphero.pack(fill="x", expand=True, padx=25)
            self.frame_settings.pack(fill="x", expand=True, padx=25)
            self.frame_tests.pack(fill="x", expand=True, padx=25)
            self.frame_maze_settings.pack(fill="x", expand=True, padx=25)
            self.frame_quit.pack(fill="x", expand=True, padx=25)

    def create_main_window(self):
        """ Sets the title and dimensions for main window

        :return: nothing
        """
        self.master.title("Sphero Maze Runner")
        self.master.minsize(width=500, height=200)
        self.master.maxsize(1000, 1000)

    def create_widgets(self):
        """
        Calls functions to create and pack widgets for the GUI's various
        features. Set flag to true when complete.

        :return: nothing
        """
        self.pack_inner_title()
        self.pack_sphero()
        self.pack_settings()
        self.pack_tests()
        self.pack_maze()
        self.pack_quit()

        self.widgets_setup = True

    def pack_inner_title(self):
        """ The main title is found at the top of the GUI. This is simply the
        title of the program.

        :return: nothing
        """
        inner_title = tk.Label(self.gui_frames.frame_inner_title,
                               font=('Rockwell', 26), borderwidth=2)
        inner_title["text"] = "Sphero Maze Runner"
        inner_title.pack(side="top")

    def pack_sphero(self):
        """Creates and packs the Sphero Connection section. Contains the sphero
         widgets (Connect, disconnect, select sphero)

        Begins by creates secondary and tertiary tkinter frames for organizing
        buttons and labels.  Next the title, buttons and labels are packed.
        :return:
        """
        # Create and pack Secondary frames
        sec_frame_sphero_title = tk.Frame(self.gui_frames.frame_sphero)
        sec_frame_sphero_status = tk.Frame(self.gui_frames.frame_sphero)
        sec_frame_sphero_interface = tk.Frame(self.gui_frames.frame_sphero)
        sec_frame_sphero_title.pack()
        sec_frame_sphero_status.pack()
        sec_frame_sphero_interface.pack(expand=True)

        # Create and pack Tertiary frames
        ter_frame_sphero_connect = tk.Frame(sec_frame_sphero_interface)
        ter_frame_sphero_select = tk.Frame(sec_frame_sphero_interface,
                                           width=500)
        ter_frame_sphero_disconnect = tk.Frame(sec_frame_sphero_interface)
        ter_frame_sphero_connect.pack(side="left", padx=5,
                                      fill="x", expand=True)
        ter_frame_sphero_select.pack(side="left", padx=50,
                                     fill="x", expand=True)
        ter_frame_sphero_disconnect.pack(side="left", padx=5,
                                         fill="x", expand=True)

        # Create and pack title
        sphero_title = tk.Label(sec_frame_sphero_title,
                                text="Sphero Connection",
                                font=('Arial', 16), fg='blue')
        sphero_title.pack()

        # Create and pack status
        self.sphero_status = tk.StringVar()
        self.sphero_status_display =\
            tk.Label(sec_frame_sphero_status, font=('Arial', 12))
        self.sphero_status_display.pack()
        self.sphero_status_display["textvariable"] = self.sphero_status
        self.sphero_status.set("Disconnected")
        self.sphero_status_display.configure(fg="red")

        # Create and pack buttons and selection
        sphero_connect_button =\
            tk.Button(ter_frame_sphero_connect, text="Connect",
                      font=('system', 14), fg="green",
                      command=lambda:
                      self.app.sphero_connect(self.sphero_device.get()),
                      width=9, height=2)
        sphero_disconnect_button = \
            tk.Button(ter_frame_sphero_disconnect, text="Disconnect",
                      font=('system', 14), fg="red",
                      command=self.app.sphero_disconnect,
                      width=9, height=2)

        # Menu for selecting which Sphero to connect to
        sphero_menu_options = ['Sphero-BPW', 'Sphero-WYO',
                               'Sphero-GWR', 'Sphero-WPW']
        sphero_select_menu = OptionMenu(ter_frame_sphero_select,
                                        self.sphero_device,
                                        *sphero_menu_options)
        sphero_orient_button =\
            tk.Button(ter_frame_sphero_select, text="Orientation",
                      font=('system', 8), fg="purple",
                      command=self.app.sphero_orient, width=13, height=1)

        # Pack it all up
        sphero_connect_button.pack()
        sphero_select_menu.pack()
        sphero_disconnect_button.pack()
        sphero_orient_button.pack()

    def pack_settings(self):
        """ Contains the setting widgets for the filter, camera, and PID
         controller

        :return: nothing
        """
        # Create and pack the title label
        title_label = tk.Label(self.gui_frames.frame_settings, text="Settings",
                               font=("Arial", 14), fg='blue')
        title_label.pack(pady=1)
        # Create and pack secondary frames
        sec_frame_filter = tk.Frame(self.gui_frames.frame_settings)
        sec_frame_corners = tk.Frame(self.gui_frames.frame_settings)
        sec_frame_camera = tk.Frame(self.gui_frames.frame_settings)
        sec_frame_pid = tk.Frame(self.gui_frames.frame_settings)
        sec_frame_filter.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_corners.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_camera.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_pid.pack(side="left", padx=5, fill="x", expand=True)

        # Filter Button
        filter_button = tk.Button(sec_frame_filter, text="Filters",
                                  command=self.app.calibrate_filters,
                                  font=('system', 14),
                                  fg="deepskyblue", width=12, height=2)
        filter_button.pack()

        # Corners Button
        corners_button = tk.Button(sec_frame_corners, text="Corners",
                                   command=self.app.calibrate_corners,
                                   font=('system', 14),
                                   fg="light slate blue", width=12, height=2)
        corners_button.pack()

        # Camera Button
        camera_button = tk.Button(sec_frame_camera, text="Camera",
                                  command=self.app.calibrate_camera,
                                  font=('system', 14),
                                  fg="dark orange", width=12, height=2)
        camera_button.pack()

        # PID button
        pid_button = tk.Button(sec_frame_pid, text="PID",
                               command=self.app.controller_set_pid,
                               font=('system', 14),
                               fg="palevioletred", width=12, height=2)
        pid_button.pack()

    #
    def pack_tests(self):
        """Contains the setting widgets for the live feed, maze walls test, and
         Sphero location test.

        :return:
        """
        # Create and pack the title label
        title_label = tk.Label(self.gui_frames.frame_tests,
                               text="Tests", font=("Arial", 14), fg='blue')
        title_label.pack(pady=1)

        # Create and pack secondary frames
        sec_frame_mazefeed = tk.Frame(self.gui_frames.frame_tests)
        sec_frame_livefeed = tk.Frame(self.gui_frames.frame_tests)
        sec_frame_spherofeed = tk.Frame(self.gui_frames.frame_tests)
        sec_frame_mazefeed.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_livefeed.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_spherofeed.pack(side="left", padx=5, fill="x", expand=True)

        # Live Feed Button
        live_feed_button = tk.Button(sec_frame_livefeed, text="Live Feed",
                                     command=self.app.toggle_live_feed,
                                     font=('system', 14), fg="seagreen",
                                     width=12, height=2)

        live_feed_button.pack()

        # Live Feed Button
        maze_feed_button = tk.Button(sec_frame_mazefeed, text="Maze Walls",
                                     command=self.app.toggle_maze_feed,
                                     font=('system', 14),
                                     fg="cornflowerblue", width=12, height=2)

        maze_feed_button.pack()

        # Live Feed Button
        sphero_feed_button = tk.Button(sec_frame_spherofeed,
                                       text="Sphero Position",
                                       command=self.app.toggle_sphero_feed,
                                       font=('system', 14), fg="dark orchid",
                                       width=12, height=2)

        sphero_feed_button.pack()

    def pack_maze(self):
        """ Contains the widgets for the maze start and stop features.
         controller

        :return: nothing
        """
        # Create and pack secondary frames
        sec_frame_maze_info = tk.Frame(self.gui_frames.frame_maze_settings)
        sec_frame_maze_buttons = tk.Frame(self.gui_frames.frame_maze_settings)

        sec_frame_maze_info.pack()
        sec_frame_maze_buttons.pack()

        # Create and pack the title
        maze_settings_title = tk.Label(sec_frame_maze_info, text="Maze Status",
                                       font=('Arial', 16), fg="blue")
        maze_settings_title.pack(pady=1)

        # Create and pack the maze status

        self.maze_settings_display = tk.Label(sec_frame_maze_info,
                                              font=('Arial', 10), fg="red",
                                              text="Maze Status: STOPPED")
        self.maze_settings_display.pack()

        # Create and pack the start maze button
        maze_start_button = tk.Button(sec_frame_maze_buttons,
                                      command=self.app.maze_start,
                                      text="START", font=("Rockwell", 14),
                                      fg="green", width=12, height=2)
        maze_stop_button = tk.Button(sec_frame_maze_buttons,
                                     command=self.app.maze_stop,
                                     text="STOP", font=("Rockwell", 14),
                                     fg="red", width=12, height=2)
        maze_start_button.pack(pady=1, padx=10, side="left")
        maze_stop_button.pack(pady=1, padx=10, side="left")

    def pack_quit(self):
        """
        Packs the quit button
        :return: nothing
        """
        quitbutton = tk.Button(self.gui_frames.frame_quit, text="QUIT",
                               font=('Arial', 16), fg="red",
                               command=self.close_window, borderwidth=5)
        quitbutton.pack(side="top", pady=1, padx=10, expand=True)

    def close_window(self):
        """ Closes the GUI, calls on the main program to quit.

        :return: nothing
        """
        self.app.quit_program()
        time.sleep(0.01)
        self.master.destroy()
        self.master.quit()

    def update_status_indicators(self):
        """ Update the sphero connection flag.  Called by main program

        :return: nothing
        """
        if self.update_sphero_connection:
            if not self.sphero_device_connected:
                self.sphero_status.set("Connected: %s" %
                                       self.sphero_device.get())
                self.sphero_status_display.configure(fg="green")
                self.sphero_device_connected = True
            else:
                self.sphero_status.set("Disconnected")
                self.sphero_status_display.configure(fg="red")
                self.sphero_device_connected = False

            self.update_sphero_connection = False

    def sphero_connection_changed(self):
        """ Change a flag to indicate that the connection needs to change.

        :return: nothing
        """
        self.update_sphero_connection = True


# This class creates a window used to calibrate the camera settings.
class CalibrateWindow:
    """
    GUI for adjusting camera settings. Its two sections set the camera values of
    brightness and exposure.

    """
    def __init__(self, master, cam):
        """ Creates the widgets for adjusting camera brightness and exposure.



        :param master: Tk root
        :param cam: Camera object (from camera_main)
        """
        self.master = master
        self.maze_camera = cam

        # Initialize size of window
        self.master.title("Camera Settings")
        self.master.minsize(width=500, height=250)
        self.master.maxsize(800, 800)

        # Camera Settings, these values
        self.setting_exposure = self.maze_camera.cam_exposure_value
        self.setting_brightness = self.maze_camera.cam_brightness_value

        # GUI variables
        self.exposure_value = tk.StringVar()
        self.exposure_value_display = tk.Label()
        self.brightness_value = tk.StringVar()
        self.brightness_value_display = tk.Label()

        # Flag used for live stream
        self.adjusting = True

        # Set incrementer for brightness and exposure (OS dependent)
        if self.maze_camera.OS == 'Windows':
            self.incrementer = 1  # Windows has smaller increments than Linux
        else:
            self.incrementer = 5

        # Initialize and pack the frames
        self.gui_frames = self.Frames(self.master)

        # Pack each of the setting interfaces
        self.exposure_pack()
        self.brightness_pack()
        # Pack the quit button
        self.quit_button_pack()

    class Frames:
        def __init__(self, master):
            self.frame_exposure = tk.Frame(master)
            self.frame_brightness = tk.Frame(master)
            self.frame_quit = tk.Frame(master)

            self.frame_exposure.pack(side="top", fill="x", expand=True)
            self.frame_brightness.pack(side="top", fill="x", expand=True)
            self.frame_quit.pack(side="bottom", fill="x", expand=True)

    '''# Frame for each part
    def frames_init(self):
        self.frame_exposure = tk.Frame(self.master)
        self.frame_brightness = tk.Frame(self.master)
        self.frame_quit = tk.Frame(self.master)

    # Pack each frame
    def frames_pack(self):
        self.frame_exposure.pack(side="top",fill="x", expand=True)
        self.frame_brightness.pack(side="top",fill="x", expand=True)
        self.frame_quit.pack(side="bottom",fill="x", expand=True)'''

    def exposure_pack(self):
        """ Exposure frames, buttons, and variables

        :return: nothing
        """
        # Create and pack sub frames
        frame_exposure_add = tk.Frame(self.gui_frames.frame_exposure,
                                      borderwidth=3)
        frame_exposure_info = tk.Frame(self.gui_frames.frame_exposure,
                                       relief="sunken", borderwidth=3)
        frame_exposure_subtract = tk.Frame(self.gui_frames.frame_exposure,
                                           borderwidth=3)

        frame_exposure_subtract.pack(side="left", fill="x",
                                     expand=True, padx=3)
        frame_exposure_info.pack(side="left", fill="both", expand=True)
        frame_exposure_add.pack(side="left", fill="x", expand=True, padx=3)

        # Display the exposure value
        self.exposure_value_display = tk.Label(frame_exposure_info,
                                               font=('Arial', 10), pady=5)
        self.exposure_value_display["textvariable"] = self.exposure_value

        exposure_add_button =\
            tk.Button(frame_exposure_add, text="+",
                      font=('Arial', 16), fg="green",
                      command=lambda:
                      self.maze_camera.set_exposure(self.__set_exposure(True)))
        exposure_label = tk.Label(frame_exposure_info, text="Exposure",
                                  font=('Arial', 16), fg="blue", width=20)
        exposure_sub_button =\
            tk.Button(frame_exposure_subtract, text="-",
                      font=('Arial', 16), fg="red",
                      command=lambda:
                      self.maze_camera.set_exposure(self.__set_exposure(False)))

        exposure_sub_button.pack(side="top", fill="both", expand=True)
        exposure_label.pack(side="top")
        self.exposure_value_display.pack(side="top")
        exposure_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current exposure value
        self.exposure_value.set(self.maze_camera.cam_exposure_value)

    def brightness_pack(self):
        """ Brightness frames, variables, and buttons

        :return: nothing
        """
        # Create and pack sub frames
        frame_brightness_add = tk.Frame(self.gui_frames.frame_brightness,
                                        borderwidth=3)
        frame_brightness_info = tk.Frame(self.gui_frames.frame_brightness,
                                         relief="sunken", borderwidth=3)
        frame_brightness_subtract =\
            tk.Frame(self.gui_frames.frame_brightness, borderwidth=3)

        frame_brightness_subtract.pack(side="left", fill="x",
                                       expand=True, padx=3)
        frame_brightness_info.pack(side="left", fill="both",
                                   expand=True)
        frame_brightness_add.pack(side="left", fill="x",
                                  expand=True, padx=3)

        # Exposure frame variable (frame_exposure_value) stores the exposure as
        # collected from the camera object
        self.brightness_value_display = tk.Label(frame_brightness_info,
                                                 font=('Arial', 10), pady=5)
        self.brightness_value_display["textvariable"] = self.brightness_value

        brightness_add_button =\
            tk.Button(frame_brightness_add, text="+",
                      font=('Arial', 16), fg="green",
                      command=lambda: self.maze_camera.set_brightness(
                          self.__set_brightness(True)))
        brightness_label = tk.Label(frame_brightness_info,
                                    text="Brightness", font=('Arial', 16),
                                    fg="blue", width=20)
        brightness_sub_button =\
            tk.Button(frame_brightness_subtract, text="-",
                      font=('Arial', 16), fg="red",
                      command=lambda: self.maze_camera.set_brightness(
                          self.__set_brightness(False)))

        brightness_sub_button.pack(side="top", fill="both", expand=True)
        brightness_label.pack(side="top")
        self.brightness_value_display.pack(side="top")
        brightness_add_button.pack(side="top", fill="both", expand=True)

        # Initialize Brightness Value
        self.brightness_value.set(self.setting_brightness)

    def quit_button_pack(self):
        """
        Quit button
        :return: nothing
        """
        quitbutton = tk.Button(self.gui_frames.frame_quit, text="DONE",
                               font=('Arial', 16), fg="red",
                               command=self.close_windows, borderwidth=5)
        quitbutton.pack(side="top", fill="x", expand=True)

    #
    def __set_exposure(self, increment):
        """ This function interacts with the camera class to set the exposure of
         the camera 'increment' is a boolean, where if increment == True, the
          exposure is increased, otherwise, it is decremented

          Check to see if max or min is reached.  If so, stay at max or min,
          otherwise increment or decrement as needed value

        :param increment: flag. True increments, false decrements
        :return: nothing
        """
        # Increment Exposure
        if increment:
            #
            if self.maze_camera.cam_exposure_value >=\
                    self.maze_camera.cam_exposure_max:
                self.maze_camera.cam_exposure_value =\
                    self.maze_camera.cam_exposure_max
            else:
                self.maze_camera.cam_exposure_value += self.incrementer
            self.exposure_value.set(self.maze_camera.cam_exposure_value)

        # Decrement Exposure
        elif not increment:
            # Check to see if at min.
            if self.maze_camera.cam_exposure_value <=\
                    self.maze_camera.cam_exposure_min:
                self.maze_camera.cam_exposure_value =\
                    self.maze_camera.cam_exposure_min
            else:
                self.maze_camera.cam_exposure_value -= self.incrementer
            self.exposure_value.set(self.maze_camera.cam_exposure_value)

        return self.maze_camera.cam_exposure_value  # Return value

    def __set_brightness(self, increment):
        """ This function interacts with the camera class to set the brightness
         of the camera 'increment' is a boolean, where if increment == True, the
         brightness is increased, otherwise it is decremented

         Check to see if max or min is reached.  If so, stay at max or min,
          otherwise increment or decrement as needed value

        :param increment: flag. True increments, false decrements
        :return: nothing
        """
        # Increment Brightness
        if increment:
            if self.maze_camera.cam_brightness_value >=\
                    self.maze_camera.cam_brightness_max:
                self.maze_camera.cam_brightness_value =\
                    self.maze_camera.cam_brightness_max
            else:
                self.maze_camera.cam_brightness_value += self.incrementer
            self.brightness_value.set(self.maze_camera.cam_brightness_value)
        # Decrement Brightness
        elif not increment:
            if self.maze_camera.cam_brightness_value <=\
                    self.maze_camera.cam_brightness_min:
                self.maze_camera.cam_brightness_value =\
                    self.maze_camera.cam_brightness_min
            else:
                self.maze_camera.cam_brightness_value -= self.incrementer
            self.brightness_value.set(self.maze_camera.cam_brightness_value)

        return self.maze_camera.cam_brightness_value  # Return value

    def close_windows(self):
        """
        Closes window
        :return: nothing
        """
        self.maze_camera.save_cam_settings()
        self.adjusting = False
        time.sleep(0.1)
        self.master.destroy()


class PIDWindow:
    """
    This class creates a window used to change the PID controller values.  It
    consists of three sections where each section has incrementer and
    decrementer buttons and displays the value of the gain.
    """
    def __init__(self, master, controller):
        """ Creates the widgets for controlling the PID gain values.

        :param master: tkinter root
        :param controller: Controller object
        """
        self.master = master
        self.controller = controller

        # Initialize size of window
        self.master.title("PID Settings")
        self.master.minsize(width=500, height=250)
        self.master.maxsize(800, 800)

        # Initialize and pack the frames
        self.gui_frames = self.Frames(self.master)

        # StringVariables for PID values
        self.kp_value = tk.StringVar()
        self.kp_value_display = tk.Label()
        self.ki_value = tk.StringVar()
        self.ki_value_display = tk.Label()
        self.kd_value = tk.StringVar()
        self.kd_value_display = tk.Label()

        # Pack each of the setting interfaces
        self.kp_pack()
        self.ki_pack()
        self.kd_pack()

        # Pack the quit button
        self.quit_button_pack()

    class Frames:
        def __init__(self, master):
            """ Creates and packs frames for the widgets

            :param master: tkinter root
            """
            self.frame_kp = tk.Frame(master)
            self.frame_ki = tk.Frame(master)
            self.frame_kd = tk.Frame(master)
            self.frame_quit = tk.Frame(master)

            self.frame_kp.pack(side="top", fill="x", expand=True)
            self.frame_ki.pack(side="top", fill="x", expand=True)
            self.frame_kd.pack(side="top", fill="x", expand=True)
            self.frame_quit.pack(side="bottom", fill="x", expand=True)

    def kp_pack(self):
        """
        Kp frames, buttons, and variables

        :return: nothing
        """
        # Create and pack sub frames
        frame_kp_add = tk.Frame(self.gui_frames.frame_kp, borderwidth=3)
        frame_kp_info = tk.Frame(self.gui_frames.frame_kp, relief="sunken",
                                 borderwidth=3)
        frame_kp_subtract = tk.Frame(self.gui_frames.frame_kp, borderwidth=3)

        frame_kp_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_kp_info.pack(side="left", fill="both", expand=True)
        frame_kp_add.pack(side="left", fill="x", expand=True, padx=3)

        # Kp frame variable (frame_kp_value) stores the Kp from the maze
        self.kp_value_display = tk.Label(frame_kp_info,
                                         font=('Arial', 10), pady=5)
        self.kp_value_display["textvariable"] = self.kp_value

        kp_add_button = tk.Button(frame_kp_add, text="+",
                                  font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_kp(True))
        kp_label = tk.Label(frame_kp_info, text="Proportional Constant",
                            font=('Arial', 16), fg="blue", width=20)
        kp_sub_button = tk.Button(frame_kp_subtract, text="-",
                                  font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_kp(False))

        kp_sub_button.pack(side="top", fill="both", expand=True)
        kp_label.pack(side="top")
        self.kp_value_display.pack(side="top")
        kp_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current Kp value
        self.kp_value.set(self.controller.KP_gain)

    def ki_pack(self):
        """
        Ki frames, buttons, and variables

        :return: nothing
        """
        # Create and pack sub frames
        frame_ki_add = tk.Frame(self.gui_frames.frame_ki, borderwidth=3)
        frame_ki_info = tk.Frame(self.gui_frames.frame_ki, relief="sunken",
                                 borderwidth=3)
        frame_ki_subtract = tk.Frame(self.gui_frames.frame_ki, borderwidth=3)

        frame_ki_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_ki_info.pack(side="left", fill="both", expand=True)
        frame_ki_add.pack(side="left", fill="x", expand=True, padx=3)

        # Ki frame variable (frame_ki_value) stores the ki from the maze
        self.ki_value_display = tk.Label(frame_ki_info,
                                         font=('Arial', 10), pady=5)
        self.ki_value_display["textvariable"] = self.ki_value
        ki_add_button = tk.Button(frame_ki_add, text="+",
                                  font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_ki(True))
        ki_label = tk.Label(frame_ki_info, text="Integral Constant",
                            font=('Arial', 16), fg="blue", width=20)
        ki_sub_button = tk.Button(frame_ki_subtract, text="-",
                                  font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_ki(False))

        ki_sub_button.pack(side="top", fill="both", expand=True)
        ki_label.pack(side="top")
        self.ki_value_display.pack(side="top")
        ki_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current ki value
        self.ki_value.set(self.controller.KI_gain)

    def kd_pack(self):
        """
        Kd frames, buttons, and variables

        :return: nothing
        """
        # Create and pack sub frames
        frame_kd_add = tk.Frame(self.gui_frames.frame_kd, borderwidth=3)
        frame_kd_info = tk.Frame(self.gui_frames.frame_kd,
                                 relief="sunken", borderwidth=3)
        frame_kd_subtract = tk.Frame(self.gui_frames.frame_kd, borderwidth=3)

        frame_kd_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_kd_info.pack(side="left", fill="both", expand=True)
        frame_kd_add.pack(side="left", fill="x", expand=True, padx=3)

        # Kd frame variable (frame_kd_value) stores the Kd from the maze
        self.kd_value_display = tk.Label(frame_kd_info,
                                         font=('Arial', 10), pady=5)
        self.kd_value_display["textvariable"] = self.kd_value

        kd_add_button = tk.Button(frame_kd_add, text="+",
                                  font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_kd(True))
        kd_label = tk.Label(frame_kd_info, text="Derivative Constant",
                            font=('Arial', 16), fg="blue", width=20)
        kd_sub_button = tk.Button(frame_kd_subtract, text="-",
                                  font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_kd(False))

        kd_sub_button.pack(side="top", fill="both", expand=True)
        kd_label.pack(side="top")
        self.kd_value_display.pack(side="top")
        kd_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current Kd value
        self.kd_value.set(self.controller.KD_gain)

    def quit_button_pack(self):
        """
        Quit button

        :return: nothing
        """
        quitbutton = tk.Button(self.gui_frames.frame_quit,
                               text="DONE", font=('Arial', 16), fg="red",
                               command=self.close_windows, borderwidth=5)
        quitbutton.pack(side="top", fill="x", expand=True)

    def __set_kp(self, increment):
        """ Updates and sets kp value.

        :param increment: flag determines whether to increment of decrement
        :return: nothing
        """
        if increment:
            self.controller.KP_gain += 1
        elif not increment:
            self.controller.KP_gain -= 1
            # Do not drop below zero
            if self.controller.KP_gain <= 0:
                self.controller.KP_gain = 0

        self.kp_value.set(self.controller.KP_gain)

    def __set_ki(self, increment):
        """ Updates and sets ki value.

        :param increment: flag determines whether to increment of decrement
        :return: nothing
        """
        if increment:
            self.controller.KI_gain += 1
        elif not increment:
            self.controller.KI_gain -= 1
            # Do not drop below zero
            if self.controller.KI_gain <= 0:
                self.controller.KI_gain = 0

        self.ki_value.set(self.controller.KI_gain)

    def __set_kd(self, increment):
        """ Updates and sets kd value.

        :param increment: flag determines whether to increment of decrement
        :return: nothing
        """
        if increment:
            self.controller.KD_gain += 1
        elif not increment:
            self.controller.KD_gain -= 1
            # Do not drop below zero
            if self.controller.KD_gain <= 0:
                self.controller.KD_gain = 0
                return

        self.kd_value.set(self.controller.KD_gain)

    def close_windows(self):
        """
        Closes window
        :return: nothing
        """
        self.controller.save_pid()
        time.sleep(0.1)
        self.master.destroy()
